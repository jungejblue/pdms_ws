"""NAVSIM v1.1 aggregation and submetric semantics, with ETRI geometry adapters."""
import numpy as np
from shapely.geometry import Point,LineString
from shapely.affinity import translate
from .geometry import ego_box,center
from .comfort import ego_is_comfortable

AGENTS={'car','vehicle','pedestrian','cyclist','bicycle','motorcycle','bus','truck'}

def normalize_pair(progress,nc,dac):
    mask=np.asarray(nc)*np.asarray(dac); raw=np.asarray(progress)*mask
    if raw.max()>5.: return raw/raw.max()
    out=np.ones(len(raw));out[mask==0]=0;return out

def relative_angle(state,xy):
    delta=np.asarray(xy)-state[:2]
    if np.linalg.norm(delta)<1e-8: return np.pi/2
    return abs(np.arctan2(np.sin(np.arctan2(delta[1],delta[0])-state[2]),np.cos(np.arctan2(delta[1],delta[0])-state[2])))

def score_rollout(roll,sample,cfg):
    states=roll['states'];controls=roll['controls'];veh=cfg.vehicle
    footprints=[ego_box(s,veh) for s in states]
    # NAVSIM v1 corner membership semantics, additionally record full footprint diagnostic.
    corner_off=[];multi=[]
    for poly in footprints:
        corners=[Point(p) for p in list(poly.exterior.coords)[:4]]
        corner_off.append(not all(sample['drivable'].contains(p) for p in corners))
        in_lanes=np.array([[lane.contains(p) for p in corners] for lane in sample['lanes']])
        multi.append(bool((in_lanes.any(axis=1).sum()>1) and not in_lanes.all(axis=1).any()))
    dac=float(not any(corner_off));nc=1.;events=[]
    ignored=set() # Initial overlaps are classified, not silently discarded.
    for t,(state,poly) in enumerate(zip(states,footprints)):
        for obj in sample['objects'][t]:
            if obj['id'] in ignored or not poly.intersects(obj['polygon']): continue
            angle=relative_angle(state,obj['xy'])
            if state[3]<=.05: kind='stopped_ego';fault=False
            elif obj['speed']<=.05: kind='stopped_track';fault=True
            elif angle>np.deg2rad(150): kind='rear';fault=False
            elif LineString([poly.exterior.coords[0],poly.exterior.coords[3]]).intersects(obj['polygon']): kind='front';fault=True
            else: kind='lateral';fault=bool(multi[t] or corner_off[t])
            events.append({'metric':'NC','time_s':round(t*.1,2),'track_id':obj['id'],'kind':kind,'at_fault':fault})
            if fault: nc=min(nc,0. if obj['class'].lower() in AGENTS else .5)
            else: ignored.add(obj['id'])
    ttc=1.;ttc_ignored=set()
    for t,(state,poly) in enumerate(zip(states,footprints)):
        if state[3]<.005: continue
        intersect=sample['intersection'].contains(Point(state[:2]))
        for offset in (0,3,6,9):
            projected=translate(poly,xoff=state[3]*np.cos(state[2])*offset*.1,yoff=state[3]*np.sin(state[2])*offset*.1)
            for obj in sample['objects'][t+offset]:
                if obj['id'] in ttc_ignored or not projected.intersects(obj['polygon']):continue
                angle=relative_angle(state,obj['xy'])
                if angle<np.deg2rad(30) or ((multi[t] or corner_off[t] or intersect) and angle<=np.deg2rad(150)):
                    ttc=0.;events.append({'metric':'TTC','time_s':round(t*.1,2),'lookahead_s':offset*.1,'track_id':obj['id']})
                else: ttc_ignored.add(obj['id'])
    comfort_states=np.zeros((1,31,6));comfort_states[0,:,:4]=states
    # Body-frame acceleration; initial finite control extension is documented.
    accel=np.r_[controls[:,1],controls[-1,1]]
    steer=np.r_[controls[:,0],controls[-1,0]]
    comfort_states[0,:,4]=accel
    comfort_states[0,:,5]=states[:,3]**2/cfg.vehicle.wheelbase*np.tan(steer)
    checks=ego_is_comfortable(comfort_states,np.arange(31)*.1)[0]
    c=float(checks.all())
    route=sample['route']
    projected=[route.project(Point(center(s,veh))) for s in states]
    progress=max(0.,projected[-1]-projected[0])
    if any(corner_off): events.append({'metric':'DAC','time_s':round(corner_off.index(True)*.1,2)})
    return {'NC':float(nc),'DAC':dac,'TTC':ttc,'C':c,'progress_m':progress,
            'comfort_checks':dict(zip(['lon_accel','lat_accel','jerk_magnitude','lon_jerk','yaw_accel','yaw_rate'],map(bool,checks))),
            'events':events,'full_footprint_DAC':float(all(sample['drivable'].covers(p) for p in footprints)),
            'route_projection_max_distance':float(max(route.distance(Point(center(s,veh))) for s in states)),
            'route_endpoint_clipped':bool(projected[-1]>=route.length-1e-5)}

def score_pair(gt,model,sample,cfg):
    scores=[score_rollout(r,sample,cfg) for r in (gt,model)]
    eps=normalize_pair([s['progress_m'] for s in scores],[s['NC'] for s in scores],[s['DAC'] for s in scores])
    for s,ep in zip(scores,eps):
        s['EP']=float(ep);s['PDMS']=s['NC']*s['DAC']*(5*s['EP']+5*s['TTC']+2*s['C'])/12
    return scores
