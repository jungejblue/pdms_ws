from pathlib import Path
import json
import numpy as np
import pandas as pd
from .prediction import load_pickle
from .initial_speed import initial_speed
from .geometry import local,box,read_map,choose_route

SCALE={'s':1.,'ms':1e-3,'us':1e-6,'ns':1e-9}
def seconds(x,unit): return np.asarray(x,float)*SCALE[unit]

def resample(df,t,cols,cfg,angles=()):
    ts=seconds(df['timestamp'],cfg.raw_timestamp_unit)
    order=np.argsort(ts);ts=ts[order];values=df[cols].to_numpy(float)[order]
    if np.any(np.diff(ts)<=0): raise ValueError('duplicate/nonmonotone timestamps')
    if min(t)<ts[0]-1e-5 or max(t)>ts[-1]+1e-5: raise ValueError('insufficient future/history coverage')
    idx=np.clip(np.searchsorted(ts,t),1,len(ts)-1)
    if np.any(ts[idx]-ts[idx-1]>cfg.max_time_gap_s): raise ValueError('timestamp gap')
    for col in angles: values[:,cols.index(col)]=np.unwrap(values[:,cols.index(col)])
    return np.column_stack([np.interp(t,ts,values[:,i]) for i in range(len(cols))])

class Dataset:
    def __init__(self,root,infos,cfg):
        self.root=Path(root).expanduser().resolve() if root else None
        self.cfg=cfg;self.cache={}
        from .inputs import load_infos
        entries, self.input_report = load_infos(infos)
        self.infos={}
        for i in entries:
            if not isinstance(i,dict) or 'token' not in i or 'timestamp' not in i:
                raise ValueError('Each info requires token and timestamp (scene_token recommended)')
            token=str(i['token'])
            if token in self.infos: raise ValueError(f'Duplicate info token: {token}')
            self.infos[token]=i
        self.scene_names={self.scenario_for(token) for token in self.infos}
        if self.root is None:
            raise ValueError('Supply --data-root pointing to the original ETRI scenario/parquet directory.')
    def scenario_for(self,token):
        info=self.infos[token]
        return str(info.get('scene_token',token.rsplit('_',1)[0]))
    def scene(self,name):
        if name in self.cache:return self.cache[name]
        tables=('ego_pose','hd_ego_pose','object','hd_map')
        folder=self.root/name
        if not folder.is_dir():
            # Flat single-scene layout only. Never use the wrong scene silently.
            if len(self.scene_names)==1:folder=self.root
            else:raise FileNotFoundError(f'Scenario directory missing: {folder}')
        def file(n):
            found=[p for p in (folder/n,folder/'meta'/n) if p.is_file()]
            if not found:raise FileNotFoundError(f'{name}: {n} missing under {folder}')
            return found[0]
        self.cache[name]=(folder,{key:pd.read_parquet(file(key+'.parquet')) for key in tables})
        return self.cache[name]
    def extras(self,name,folder):
        polygons=folder/'map_polygons.geojson';routes=folder/'route_ids.json'
        return (json.loads(polygons.read_text()) if polygons.is_file() else None,
                json.loads(routes.read_text()) if routes.is_file() else {})
    def sample(self,token):
        cfg=self.cfg;info=self.infos[token];name=self.scenario_for(token);folder,data=self.scene(name)
        t0=float(seconds(info['timestamp'],cfg.info_timestamp_unit));times=t0+np.arange(31)*.1
        poses=resample(data['ego_pose'],times,['x','y','yaw'],cfg,['yaw']); origin=poses[0].copy()
        poses[:,:2]=local(poses[:,:2],origin);poses[:,2]-=origin[2]
        # Pose-based initial speed; forward difference only at the scene start.
        raw_times=seconds(data['ego_pose']['timestamp'],cfg.raw_timestamp_unit)
        speed,speed_info=initial_speed(t0,float(raw_times.min()),float(raw_times.max()),
                                      lambda q:resample(data['ego_pose'],q,['x','y'],cfg),tolerance=1e-5)
        initial=np.array([0.,0.,0.,speed])
        hd_origin=resample(data['hd_ego_pose'],np.array([t0]),['x','y','yaw'],cfg,['yaw'])[0]
        override,routes=self.extras(name,folder)
        lines,lanes,drivable,intersection,map_quality=read_map(data['hd_map'],hd_origin,cfg,override)
        explicit=routes.get(token)
        ep_data={}
        if cfg.ep_reference=='gt_path':
            from .ep_path import build_gt_path
            def pose_at(q):
                p=resample(data['ego_pose'],q,['x','y','yaw'],cfg,['yaw'])
                p[:,:2]=local(p[:,:2],origin);p[:,2]-=origin[2]
                return p
            ep_data=build_gt_path(t0,float(raw_times.max()),pose_at,cfg)
            route,ids=ep_data.pop('route'),ep_data.pop('route_ids')
            ep_data.pop('route_source')
        else:
            route,ids=choose_route(lines,poses,cfg,explicit)
        objs=data['object'];ego=objs[objs['class'].str.lower()=='ego']
        object_origin=resample(ego,np.array([t0]),['x[m]','y[m]','heading[rad]'],cfg,['heading[rad]'])[0]
        # The ego rows verify log-frame availability even when zero external agents exist.
        obj_times=t0+np.arange(40)*.1
        resample(ego,obj_times,['x[m]','y[m]'],cfg)
        objects=[[] for _ in obj_times]
        external=objs[objs['class'].str.lower()!='ego']
        for track,g in external.groupby('obj_id'):
            g=g.sort_values('timestamp'); rawt=seconds(g['timestamp'],cfg.raw_timestamp_unit)
            for k,t in enumerate(obj_times):
                if t<rawt[0]-1e-5 or t>rawt[-1]+1e-5: continue
                if len(g)==1:
                    if abs(t-rawt[0])>.025:continue
                    row=g.iloc[0]
                    vals=np.array([row[x] for x in ['x[m]','y[m]','heading[rad]',cfg.object_length_column,cfg.object_width_column]],float)
                else:
                    vals=resample(g,np.array([t]),['x[m]','y[m]','heading[rad]',cfg.object_length_column,cfg.object_width_column],cfg,['heading[rad]'])[0]
                if not np.isfinite(vals).all() or min(vals[3:])<=0: raise ValueError('invalid object box')
                xy=local(vals[None,:2],object_origin)[0]; yaw=vals[2]-object_origin[2]
                if len(g)>1:
                    idx=min(max(np.searchsorted(rawt,t),1),len(g)-1)
                    speed=float(np.linalg.norm(g.iloc[idx][['x[m]','y[m]']].to_numpy(float)-g.iloc[idx-1][['x[m]','y[m]']].to_numpy(float))/(rawt[idx]-rawt[idx-1]))
                else: speed=0.
                objects[k].append({'id':str(track),'class':str(g.iloc[0]['class']),'speed':speed,'xy':xy,'yaw':yaw,'polygon':box(*xy,yaw,vals[3]/2,vals[3]/2,vals[4])})
        return dict(token=token,scenario=name,gt=poses,initial=initial,objects=objects,route=route,route_ids=ids,
                    lines=lines,lanes=lanes,drivable=drivable,intersection=intersection,map_quality=map_quality,
                    route_source='gt_recorded_vehicle_center' if cfg.ep_reference=='gt_path' else ('explicit' if explicit else 'gt_evaluation_only'),t0=t0,initial_speed_info=speed_info,**ep_data)
