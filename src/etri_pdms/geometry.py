import numpy as np
from shapely.geometry import Polygon, LineString, Point, shape
from shapely.ops import unary_union, transform
import json

def rotation(yaw): return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
def local(xy,pose): return (np.asarray(xy)-pose[:2])@rotation(pose[2])
def box(x,y,yaw,front,rear,width):
    xy=np.array([[front,width/2],[-rear,width/2],[-rear,-width/2],[front,-width/2]])
    return Polygon(xy@rotation(yaw).T+[x,y])
def ego_box(state,veh): return box(*state[:3],veh.front,veh.rear_overhang,veh.width)
def center(state,veh): return state[:2]+veh.center_offset*np.array([np.cos(state[2]),np.sin(state[2])])

def read_map(frame,pose,cfg,override=None):
    lines={}
    for _,r in frame[frame['class']=='centerline'].iterrows():
        points=np.asarray([np.asarray(p,float)[:2] for p in r['points']])
        points=local(points,pose)
        points=points[np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>1e-6]]
        if len(points)>1: lines[str(r['id'])]=LineString(points)
    if not lines: raise ValueError('No centerlines')
    if override is not None:
        data=override if isinstance(override,dict) else json.load(open(override)); lanes=[]; drive=[]; intersections=[]
        if not data.get('properties',{}).get('verified',False):
            raise ValueError('map_polygons.geojson must declare properties.verified=true after geometry review')
        for f in data['features']:
            role=f.get('properties',{}).get('role','drivable')
            geom=shape(f['geometry'])
            # GeoJSON uses planar HD-map world metres; never lon/lat.
            def convert(x,y,z=None):
                xy=local(np.column_stack([np.atleast_1d(x),np.atleast_1d(y)]),pose)
                return xy[:,0],xy[:,1]
            geom=transform(convert,geom)
            if not geom.is_valid: raise ValueError('Invalid GeoJSON geometry')
            if role=='lane': lanes.append(geom)
            if role=='drivable': drive.append(geom)
            if role=='intersection': intersections.append(geom)
        if not lanes or not drive: raise ValueError('Validated map requires lane and drivable features')
        return lines,lanes,unary_union(drive),unary_union(intersections),'validated_geojson'
    if cfg.map_mode=='validated': raise ValueError('Missing map_polygons.geojson; validated mode does not infer drivable area')
    lanes=[line.buffer(cfg.lane_width/2,cap_style=2) for line in lines.values()]
    # Intersections inferred only for diagnostic approximate mode.
    nodes={}
    for key,line in lines.items():
        for p in [line.coords[0],line.coords[-1]]:
            node=tuple(np.round(p,1));nodes[node]=nodes.get(node,0)+1
    intersections=[Point(p).buffer(cfg.lane_width) for p,n in nodes.items() if n>=3]
    return lines,lanes,unary_union(lanes),unary_union(intersections),'approximate_centerline_buffer'

def choose_route(lines,gt,cfg,explicit_ids=None,adjacency_override=None):
    """GT selects evaluation route only. This future information never reaches MPC.
    Both proposals project on this ONE fixed ordered arc-length path.
    """
    if explicit_ids:
        ids=[str(x) for x in explicit_ids]
    else:
        ids=[]
        for state in gt:
            point=Point(state[:2]); candidates=[]
            for key,line in lines.items():
                s=line.project(point); a=np.array(line.interpolate(max(0,s-.5)).coords[0]); b=np.array(line.interpolate(min(line.length,s+.5)).coords[0])
                yaw=np.arctan2(*(b-a)[::-1]); angle=abs(np.arctan2(np.sin(yaw-state[2]),np.cos(yaw-state[2])))
                if angle<np.pi/2: candidates.append((line.distance(point)+2*angle,key))
            if not candidates: raise ValueError('No heading-compatible route lane')
            key=min(candidates)[1]
            if not ids or ids[-1]!=key: ids.append(key)
        # Fill intermediate directed segments using endpoint topology.
        adjacency=adjacency_override if adjacency_override is not None else {k:[j for j,b in lines.items() if j!=k and np.linalg.norm(np.array(a.coords[-1])-b.coords[0])<=cfg.route_endpoint_tolerance] for k,a in lines.items()}
        filled=[ids[0]]
        for target in ids[1:]:
            queue=[[filled[-1]]]; found=None
            while queue:
                chain=queue.pop(0)
                if chain[-1]==target: found=chain;break
                if len(chain)>15: continue
                for nxt in adjacency[chain[-1]]:
                    if nxt not in chain: queue.append(chain+[nxt])
                if len(queue)>10000: raise ValueError('route graph search ambiguous; supply route_ids.json')
            if found is None: raise ValueError(f'Unconnected route IDs {filled[-1]} -> {target}; supply route_ids.json')
            filled+=found[1:]
        ids=filled
    # Extend beyond GT endpoint for faster model proposals, only through unique successors.
    while True:
        last=lines[ids[-1]]
        next_ids=[key for key,line in lines.items() if key not in ids and np.linalg.norm(np.asarray(last.coords[-1])-line.coords[0])<=cfg.route_endpoint_tolerance]
        if adjacency_override is not None:
            next_ids=[key for key in adjacency_override.get(ids[-1],[]) if key not in ids and key in lines]
        if len(next_ids)!=1: break
        ids.append(next_ids[0])
    xy=[]
    for key in ids:
        p=np.asarray(lines[key].coords)
        if xy:
            gap=np.linalg.norm(np.asarray(xy[-1])-p[0])
            if gap>cfg.route_endpoint_tolerance: raise ValueError('route endpoint gap')
            p=p[1:] if gap<1e-6 else p
        xy.extend(p.tolist())
    route=LineString(xy)
    if not route.is_simple: raise ValueError('Self-intersecting route needs explicit unambiguous route chain')
    return route,ids
