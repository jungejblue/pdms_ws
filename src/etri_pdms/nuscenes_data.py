"""nuScenes JSON adapter for the common 3-second GT/MPC evaluator.

Default VAD mode: LIDAR_TOP origin trajectories, rotated into ego axes. A virtual
vehicle's rear axle is placed at this tracked anchor (explicit scoring proxy).
It does NOT claim to reconstruct the physical nuScenes vehicle footprint.
"""
from collections import defaultdict, OrderedDict
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import local,box
from .inputs import digest,load_infos
from .nuscenes_map import NuScenesMap


def rotation(q):
    q=np.asarray(q,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-8:
        raise ValueError('Invalid nuScenes quaternion')
    return Rotation.from_quat(q[[1,2,3,0]]).as_matrix()


def yaw(q):
    r=rotation(q);return np.arctan2(r[1,0],r[0,0])


def interpolate(ts,values,query,max_gap,angles=()):
    ts=np.asarray(ts,float);values=np.asarray(values,float).copy();query=np.asarray(query,float)
    if len(ts)<2 or np.any(np.diff(ts)<=0) or not np.isfinite(values).all():
        raise ValueError('Invalid nuScenes time series')
    if query.min()<ts[0]-1e-6 or query.max()>ts[-1]+1e-6:
        raise ValueError('insufficient nuScenes future/history coverage')
    idx=np.clip(np.searchsorted(ts,query),1,len(ts)-1)
    exact=np.isclose(query,ts[idx],atol=1e-6,rtol=0)|np.isclose(query,ts[idx-1],atol=1e-6,rtol=0)
    if np.any((ts[idx]-ts[idx-1]>max_gap)&~exact):raise ValueError('nuScenes timestamp gap')
    for col in angles:values[:,col]=np.unwrap(values[:,col])
    return np.column_stack([np.interp(query,ts,values[:,j]) for j in range(values.shape[1])])


def agent_class(name):
    if name.startswith('human.pedestrian'):return 'pedestrian'
    if name.startswith('vehicle.bicycle'):return 'bicycle'
    if name.startswith('vehicle.motorcycle'):return 'motorcycle'
    if name.startswith('vehicle.'):return 'vehicle'
    return name


class NuScenesDataset:
    def __init__(self,root,infos,cfg):
        self.root=Path(root).expanduser().resolve();self.cfg=cfg
        self.version=cfg.nuscenes_version
        if self.version not in ('v1.0-mini','v1.0-trainval'):
            raise ValueError('nuScenes evaluation requires annotated v1.0-mini or v1.0-trainval, not test')
        self.tables={};manifest=[]
        for name in ('sample','scene','sample_data','ego_pose','calibrated_sensor','sensor','log','sample_annotation','instance','category'):
            path=self.root/self.version/(name+'.json')
            if not path.is_file():raise FileNotFoundError(f'nuScenes metadata missing: {path}')
            rows=json.loads(path.read_text())
            table={r['token']:r for r in rows}
            if len(table)!=len(rows):raise ValueError(f'Duplicate token in {path}')
            self.tables[name]=table;manifest.append({'path':str(path),'sha256':digest(path)})
        entries,infos_report=load_infos(infos)
        self.infos={entry['token']:entry for entry in entries}
        if not self.tables['sample_annotation']:raise ValueError('No nuScenes object annotations available')
        self.scene_samples=defaultdict(list)
        for s in self.tables['sample'].values():self.scene_samples[s['scene_token']].append(s)
        for items in self.scene_samples.values():items.sort(key=lambda x:x['timestamp'])
        self.lidar_key={};self.scene_lidar=defaultdict(list)
        for sd in self.tables['sample_data'].values():
            cs=self.tables['calibrated_sensor'][sd['calibrated_sensor_token']]
            sensor=self.tables['sensor'][cs['sensor_token']]
            if sensor['channel']!='LIDAR_TOP':continue
            sample=self.tables['sample'][sd['sample_token']]
            self.scene_lidar[sample['scene_token']].append(sd)
            if sd['is_key_frame']:
                if sd['sample_token'] in self.lidar_key:raise ValueError('Duplicate LIDAR_TOP keyframe')
                self.lidar_key[sd['sample_token']]=sd
        self.scene_annotations=defaultdict(list)
        for ann in self.tables['sample_annotation'].values():
            self.scene_annotations[self.tables['sample'][ann['sample_token']]['scene_token']].append(ann)
        self.maps={};self.cache=OrderedDict()
        self.input_report={'source':str(self.root),'version':self.version,'kind':'nuscenes_raw_json',
                           'selected_files':manifest,'map_files':[], 'infos_pkl_required':True,'infos_pkl':infos_report,
                           'prediction_frame':cfg.nuscenes_prediction_frame,
                           'vehicle_anchor':'virtual_rear_axle_at_'+cfg.nuscenes_prediction_frame+'_origin',
                           'object_interpolation':'linear_2hz_to_10hz_with_unwrapped_yaw_no_extrapolation'}

    def scenario_for(self,token):return self.tables['sample'].get(token,{}).get('scene_token','__unmatched__')

    def scene(self,name):
        if name in self.cache:
            self.cache.move_to_end(name);return self.cache[name]
        sd=sorted(self.scene_lidar[name],key=lambda x:x['timestamp'])
        ts=[];poses=[]
        for row in sd:
            pose=self.tables['ego_pose'][row['ego_pose_token']]
            origin=np.asarray(pose['translation'],float)
            if self.cfg.nuscenes_prediction_frame=='lidar':
                cs=self.tables['calibrated_sensor'][row['calibrated_sensor_token']]
                origin=origin+rotation(pose['rotation'])@np.asarray(cs['translation'],float)
            ts.append(row['timestamp']/1e6);poses.append([origin[0],origin[1],yaw(pose['rotation'])])
        if len(ts)!=len(set(ts)):raise ValueError('Duplicate LIDAR_TOP timestamps')
        tracks=defaultdict(list)
        for ann in self.scene_annotations[name]:tracks[ann['instance_token']].append(ann)
        prepared=[]
        for token,annotations in tracks.items():
            annotations.sort(key=lambda a:self.tables['sample'][a['sample_token']]['timestamp'])
            at=np.array([self.tables['sample'][a['sample_token']]['timestamp']/1e6 for a in annotations])
            values=np.array([[*a['translation'][:2],yaw(a['rotation']),*a['size'][:2]] for a in annotations],float)
            if not np.isfinite(values).all() or np.any(values[:,3:]<=0):raise ValueError('Invalid object size/pose')
            category=self.tables['category'][self.tables['instance'][token]['category_token']]['name']
            prepared.append((token,at,values,agent_class(category)))
        result=(np.array(ts),np.asarray(poses),prepared)
        self.cache[name]=result
        if len(self.cache)>2:self.cache.popitem(last=False)
        return result

    def sample(self,token):
        if token not in self.infos:raise ValueError('prediction token has no infos PKL match')
        if token not in self.tables['sample']:raise ValueError('infos token has no nuScenes raw sample match')
        raw=self.tables['sample'][token]
        if abs(self.infos[token]['timestamp']/1e6-raw['timestamp']/1e6)>.001:
            raise ValueError('infos timestamp does not match raw sample timestamp (nuScenes requires microseconds)')
        name=self.scenario_for(token);cfg=self.cfg
        if token not in self.lidar_key:raise ValueError('Missing LIDAR_TOP keyframe for sample')
        key=self.lidar_key[token];t0=key['timestamp']/1e6
        ts,poses,tracks=self.scene(name)
        query=t0+np.arange(31)*.1
        gt_world=interpolate(ts,poses,query,cfg.nuscenes_pose_max_gap_s,(2,))
        origin=gt_world[0].copy();gt=gt_world.copy();gt[:,:2]=local(gt[:,:2],origin);gt[:,2]-=origin[2]
        past=interpolate(ts,poses[:,:2],np.array([t0-.1,t0]),cfg.nuscenes_pose_max_gap_s)
        initial=np.array([0.,0.,0.,np.linalg.norm(past[1]-past[0])/.1])
        object_times=t0+np.arange(40)*.1
        frame_times=np.array([s['timestamp']/1e6 for s in self.scene_samples[name]])
        # Coverage is checked even for samples with no visible external objects.
        interpolate(frame_times,np.zeros((len(frame_times),1)),object_times,cfg.nuscenes_annotation_max_gap_s)
        objects=[[] for _ in object_times]
        for track,at,values,category in tracks:
            for k,t in enumerate(object_times):
                if t<at[0]-1e-6 or t>at[-1]+1e-6:continue
                if len(at)==1:
                    if abs(t-at[0])>1e-6:continue
                    v=values[0];speed=0.
                else:
                    v=interpolate(at,values,[t],cfg.nuscenes_annotation_max_gap_s,(2,))[0]
                    i=min(max(np.searchsorted(at,t),1),len(at)-1)
                    speed=float(np.linalg.norm(values[i,:2]-values[i-1,:2])/(at[i]-at[i-1]))
                xy=local(v[None,:2],origin)[0];heading=v[2]-origin[2]
                width,length=v[3:5]
                objects[k].append({'id':track,'class':category,'xy':xy,'yaw':heading,'speed':speed,
                                  'polygon':box(*xy,heading,length/2,length/2,width)})
        scene=self.tables['scene'][name];location=self.tables['log'][scene['log_token']]['location']
        if location not in self.maps:
            map_root=Path(cfg.nuscenes_map_root).expanduser() if cfg.nuscenes_map_root else self.root
            path=map_root/'maps/expansion'/f'{location}.json'
            self.maps[location]=NuScenesMap(path)
            self.input_report['map_files'].append({'path':str(path),'sha256':digest(path)})
        geometry=self.maps[location].sample(origin,gt_world,gt,cfg)
        cs=self.tables['calibrated_sensor'][key['calibrated_sensor_token']]
        return dict(token=token,scenario=name,scene_name=scene.get('name',name),gt=gt,initial=initial,
                    objects=objects,t0=t0,**geometry,
                    prediction_rotation=rotation(cs['rotation']),dataset='nuscenes',
                    anchor_assumption=self.input_report['vehicle_anchor'])

    def transform_prediction(self,pred,sample):
        if self.cfg.nuscenes_prediction_frame=='ego':return pred
        # VAD uses LiDAR-origin displacements in the current calibrated LiDAR axes.
        xyz=np.column_stack([pred,np.zeros(len(pred))])
        return (xyz@sample['prediction_rotation'].T)[:,:2]
