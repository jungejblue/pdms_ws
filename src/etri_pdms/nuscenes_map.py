"""Read nuScenes map-expansion JSON geometry; no sensor files or SDK required."""
import json
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon, LineString, Point, box
from shapely.ops import unary_union, transform
from .geometry import local, choose_route


def discretize_arcline(paths, resolution=0.5):
    """Integrate signed constant curvature segments in planar SE(2)."""
    points=[]
    for path in paths:
        pose=np.asarray(path['start_pose'],float).copy()
        radius=float(path['radius'])
        if not np.isfinite(radius) or radius<=0: raise ValueError('Invalid map arcline radius')
        shape=path['shape'];lengths=path['segment_length']
        if len(shape)!=3 or len(lengths)!=3 or any(c not in 'LSR' for c in shape):
            raise ValueError('Unsupported map arcline shape')
        points.append(pose[:2].copy())
        for char,length in zip(shape,lengths):
            length=float(length)
            if not np.isfinite(length) or length<0: raise ValueError('Invalid arcline length')
            k={'L':1.,'S':0.,'R':-1.}[char]/radius
            count=max(1,int(np.ceil(length/resolution))); ds=length/count
            for _ in range(count):
                x,y,yaw=pose; next_yaw=yaw+k*ds
                if k==0: pose[:2]=[x+ds*np.cos(yaw),y+ds*np.sin(yaw)]
                else: pose[:2]=[x+(np.sin(next_yaw)-np.sin(yaw))/k,y+(np.cos(yaw)-np.cos(next_yaw))/k]
                pose[2]=next_yaw;points.append(pose[:2].copy())
    if not points:return np.empty((0,2))
    arr=np.asarray(points);return arr[np.r_[True,np.linalg.norm(np.diff(arr,axis=0),axis=1)>1e-6]]


class NuScenesMap:
    def __init__(self, path):
        self.path=Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(f'nuScenes map expansion JSON missing: {path}; install maps/expansion/*.json')
        doc=json.loads(self.path.read_text())
        self.doc=doc;nodes={r['token']:(r['x'],r['y']) for r in doc['node']}
        polygons={}
        for record in doc['polygon']:
            outer=[nodes[t] for t in record['exterior_node_tokens']]
            holes=[[nodes[t] for t in h['node_tokens']] for h in record.get('holes',[]) if h['node_tokens']]
            polygons[record['token']]=Polygon(outer,holes)
        self.layers={}
        for layer in ('drivable_area','lane','lane_connector','road_segment'):
            values=[]
            for rec in doc.get(layer,[]):
                if layer=='road_segment' and not rec.get('is_intersection',False):continue
                tokens=rec.get('polygon_tokens',[rec.get('polygon_token')])
                for token in tokens:
                    if token is not None:values.append(polygons[token])
            self.layers[layer]=values
        if not self.layers['drivable_area'] or not self.layers['lane']:
            raise ValueError('Map expansion requires drivable_area and lane polygons')
        self.lines={}
        for token,paths in doc.get('arcline_path_3',{}).items():
            points=discretize_arcline(paths)
            if len(points)>1:self.lines[token]=LineString(points)
        if not self.lines:raise ValueError('Map expansion has no arcline_path_3 lane centerlines')
        self.connectivity=doc.get('connectivity',{})

    def sample(self,origin,gt_world,gt_local,cfg):
        # Large patch containing the full reachable 3-second rollout envelope.
        reach=cfg.controller.v_max*cfg.horizon_s+cfg.vehicle.length+20.
        lo=np.minimum(gt_world[:,:2].min(axis=0),origin[:2]-reach)
        hi=np.maximum(gt_world[:,:2].max(axis=0),origin[:2]+reach)
        patch=box(lo[0],lo[1],hi[0],hi[1])
        def to_local(g):
            def convert(x,y,z=None):
                a=local(np.column_stack([np.atleast_1d(x),np.atleast_1d(y)]),origin)
                return a[:,0],a[:,1]
            return transform(convert,g)
        def layer(name):
            result=[]
            for geom in self.layers[name]:
                if not geom.intersects(patch):continue
                if not geom.is_valid:raise ValueError(f'Invalid official {name} polygon in map patch')
                result.append(to_local(geom))
            return result
        drive=unary_union(layer('drivable_area'))
        lanes=layer('lane')+layer('lane_connector')
        if drive.is_empty or not lanes:raise ValueError('No map coverage around this sample')
        lines={k:to_local(g) for k,g in self.lines.items() if g.intersects(patch)}
        graph={k:[v for v in self.connectivity.get(k,{}).get('outgoing',[]) if v in lines] for k in lines}
        route,ids=choose_route(lines,gt_local,cfg,adjacency_override=graph)
        return dict(lines=lines,lanes=lanes,drivable=drive,
                    intersection=unary_union(layer('road_segment')),
                    route=route,route_ids=ids,map_quality='nuscenes_map_expansion',
                    route_source='gt_evaluation_only_official_connectivity')
