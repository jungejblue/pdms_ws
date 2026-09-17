import json
from types import SimpleNamespace
import numpy as np
import pytest
from shapely.geometry import LineString
from etri_pdms.config import Config
from etri_pdms.ep_path import build_gt_path,project_gt_path
from etri_pdms.sampling import select_interval
from etri_pdms.geometry import center


def make_pose(q):return np.c_[5*q,1.75*(1-np.cos(np.pi*np.minimum(q,3)/3)),np.zeros(len(q))]


def test_lane_change_and_center_anchor():
    cfg=Config();sample=build_gt_path(0,8,make_pose,cfg)
    assert sample['route_ids']==[]
    assert sample['ep_path_info']['available_seconds']==8
    assert np.allclose(sample['route'].coords[0],[cfg.vehicle.center_offset,0])
    poses=make_pose(np.arange(31)*.1)
    s,info=project_gt_path(poses,sample['route'],cfg.vehicle)
    assert s[-1]>15 and not info['endpoint_clipped']
    assert np.all(np.diff(s)>=0)


def test_fast_model_uses_extension_and_flags_record_end():
    cfg=Config();linear=lambda q:np.c_[5*q,np.zeros(len(q)),np.zeros(len(q))]
    sample=build_gt_path(0,8,linear,cfg)
    fast=linear(np.arange(31)*.2)
    s,info=project_gt_path(fast,sample['route'],cfg.vehicle)
    assert s[-1]==pytest.approx(30) and not info['endpoint_clipped']
    short=build_gt_path(0,3,linear,cfg)
    _,info=project_gt_path(fast,short['route'],cfg.vehicle)
    assert info['endpoint_clipped']


def test_stationary_and_recorded_stop():
    cfg=Config();zero=lambda q:np.zeros((len(q),3))
    sample=build_gt_path(0,5,zero,cfg)
    ss,info=project_gt_path(zero(np.arange(31)),sample['route'],cfg.vehicle)
    assert np.all(ss==0) and not info['endpoint_clipped']
    moving=zero(np.arange(31));moving[:,0]=np.linspace(0,2,31)
    with pytest.raises(ValueError,match='stationary'):project_gt_path(moving,sample['route'],cfg.vehicle)
    stop=lambda q:np.c_[np.minimum(q,2)*5,np.zeros(len(q)),np.zeros(len(q))]
    sample=build_gt_path(0,5,stop,cfg)
    ss,info=project_gt_path(stop(np.arange(31)*.1),sample['route'],cfg.vehicle)
    assert ss[-1]==pytest.approx(10) and not info['endpoint_clipped']


def test_extension_gap_truncates_without_extrapolating():
    cfg=Config()
    def poses(q):
        if max(q)>4:raise ValueError('gap')
        return np.c_[q*5,np.zeros(len(q)),np.zeros(len(q))]
    sample=build_gt_path(0,10,poses,cfg)
    assert sample['ep_path_info']['stop_reason']=='extension_pose_gap'
    assert sample['ep_path_info']['available_seconds']==4
    with pytest.raises(ValueError):build_gt_path(0,2,poses,cfg)


def test_intersection_does_not_jump_to_later_branch():
    cfg=Config();points=np.array([[0,0],[5,0],[5,5],[0,5],[0,-5]])
    points[:,0] # reference coordinates are vehicle centers
    route=LineString(points+np.array([cfg.vehicle.center_offset,0]))
    states=np.c_[np.linspace(0,3,31),np.zeros((31,2))]
    ss,info=project_gt_path(states,route,cfg.vehicle)
    assert ss[-1]==pytest.approx(3) and not info['endpoint_clipped']


def test_scene_interval_sorted_and_unmatched_retained():
    infos={f'{scene}{i}':{'timestamp':int(i*.5*1e6),'scene_token':scene} for scene in ('a','b') for i in range(8)}
    ds=SimpleNamespace(infos=infos,cfg=Config(),scenario_for=lambda t:infos[t]['scene_token'])
    tokens=list(reversed(infos))+['missing']
    selected,meta=select_interval(tokens,ds,1.5)
    assert selected==['a0','a3','a6','b0','b3','b6','missing']
    assert meta['interval_excluded_count']==10
    assert select_interval(tokens,ds,0)[0]==tokens


def test_nuscenes_disconnected_centerlines_do_not_block_gt_mode(tmp_path,monkeypatch):
    from test_cache_coordinates import common_data
    from etri_pdms.nuscenes_data import NuScenesDataset
    import etri_pdms.nuscenes_map as maps
    common_data(tmp_path)
    def fail(*a,**k):raise ValueError('Unconnected route IDs')
    monkeypatch.setattr(maps,'choose_route',fail)
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes'))
    assert ds.sample('s0')['route_source']=='gt_recorded_vehicle_center'
    ds.cfg.ep_reference='centerline'
    with pytest.raises(ValueError,match='Unconnected'):ds.sample('s0')


def test_interval_evaluation_parallel_and_exact_replay(tmp_path):
    from test_parallel import two_scenes
    from etri_pdms.evaluator import evaluate
    pred,_=two_scenes(tmp_path)
    cfg=Config(dataset='nuscenes')
    args=(pred,tmp_path/'infos.pkl',tmp_path)
    a=evaluate(*args,tmp_path/'serial15',cfg,visualize=0,workers=1,sample_interval=1.5)
    b=evaluate(*args,tmp_path/'parallel15',cfg,visualize=0,workers=2,sample_interval=1.5)
    assert a['valid']==b['valid']==2 and a['invalid']==b['invalid']==2
    one=json.loads((tmp_path/'serial15/sample_scores.json').read_text())
    two=json.loads((tmp_path/'parallel15/sample_scores.json').read_text())
    assert one==two
    assert a['ep_reference']=='gt_path'
    selection=json.loads((tmp_path/'serial15/selection.json').read_text())
    assert selection['interval_excluded_count']==2
    assert [r['token'] for r in one]==selection['evaluated_tokens']
    with pytest.raises(ValueError,match='Exact --tokens'):
        evaluate(*args,tmp_path/'invalid',cfg,tokens=['s0'],sample_interval=1.5)
