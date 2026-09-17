"""Recorded GT vehicle-center reference; no synthetic continuation or map matching."""
import numpy as np
from shapely.geometry import LineString
from .geometry import center


def build_gt_path(t0,last_time,pose_at,cfg):
    poses=[];stop='configured_time_limit'
    for step in range(int(round(cfg.ep_gt_path_seconds/.1))+1):
        if t0+step*.1>last_time+1e-6:
            stop='recording_end';break
        try:
            pose=pose_at(np.array([t0+step*.1]))[0]
            if not np.isfinite(pose).all():raise ValueError('nonfinite GT pose')
        except ValueError:
            if step<=30:raise
            stop='extension_pose_gap';break
        poses.append(pose)
    if len(poses)<31:raise ValueError('insufficient GT path coverage')
    points=np.array([center(p,cfg.vehicle) for p in poses])
    points=points[np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>1e-7]]
    if len(points)==1:points=np.repeat(points,2,axis=0)
    route=LineString(points)
    return dict(route=route,route_ids=[],route_source='gt_recorded_vehicle_center',
                ep_path_info={'reference':'gt_path','anchor':'vehicle_center',
                              'available_seconds':round((len(poses)-1)*.1,6),
                              'requested_seconds':cfg.ep_gt_path_seconds,'stop_reason':stop,
                              'length_m':route.length,'stationary':route.length<1e-6})


def project_gt_path(states,route,vehicle):
    """Use temporally continuous projections; reject unresolved branch jumps."""
    xy=np.array([center(s,vehicle) for s in states]);p=np.asarray(route.coords)
    vec=np.diff(p,axis=0);length=np.linalg.norm(vec,axis=1)
    valid=length>1e-7;p0=p[:-1][valid];vec=vec[valid];length=length[valid]
    if not len(length):
        if np.max(np.linalg.norm(xy-p[0],axis=1))>.25:
            raise ValueError('EP stationary GT reference cannot measure a moving rollout')
        return np.zeros(len(xy)),{'endpoint_clipped':False,'stationary':True,'arc_positions_m':[0.]*len(xy)}
    offsets=np.r_[0.,np.cumsum(length)[:-1]]
    result=[];clipped=False
    for i,q in enumerate(xy):
        u=np.clip(np.sum((q-p0)*vec,axis=1)/(length**2),0,1)
        projected=p0+u[:,None]*vec
        ss=offsets+u*length;dist=np.linalg.norm(projected-q,axis=1)
        bound=.5 if i==0 else 2*np.linalg.norm(xy[i]-xy[i-1])+.1
        previous=0. if i==0 else result[-1]
        allowed=np.abs(ss-previous)<=bound+1e-6
        if not allowed.any():raise ValueError('EP projection has no continuous GT branch')
        ids=np.flatnonzero(allowed)
        j=min(ids,key=lambda k:(round(float(dist[k]),8),abs(float(ss[k]-previous))))
        if dist[j]>dist.min()+.5:
            raise ValueError('EP projection ambiguous or outside continuous GT branch')
        result.append(float(ss[j]))
        if ss[j]>=route.length-1e-5 and np.dot(q-p[-1],vec[-1]/length[-1])>.05:
            clipped=True
    return np.asarray(result),{'endpoint_clipped':clipped,'stationary':False,
                               'arc_positions_m':result,'continuity_policy':'bounded_arc_length',
                               'step_allowance_m':0.1,'motion_multiplier':2.0,
                               'branch_distance_tolerance_m':0.5,'endpoint_tolerance_m':0.05}
