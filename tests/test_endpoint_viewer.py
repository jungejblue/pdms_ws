import json
from pathlib import Path
import numpy as np
import pytest
from etri_pdms.evaluator import validate_endpoints,evaluate_sample
from etri_pdms.metrics import normalize_pair
from etri_pdms.viewer import select_viewer_period
from etri_pdms.ep_path import project_gt_path
from etri_pdms.config import Config
from shapely.geometry import LineString


def test_endpoint_final_progress_and_uncertain_gt():
    cfg=Config();route=LineString([(cfg.vehicle.center_offset,0),(cfg.vehicle.center_offset+10,0)])
    def project(x):return project_gt_path(np.c_[x,np.zeros((len(x),2))],route,cfg.vehicle)
    gt,gd=project(np.linspace(0,8,31))
    model,md=project(np.linspace(0,12,31))
    assert md['endpoint_clipped'] and not gd['endpoint_clipped']
    validate_endpoints({'route_endpoint_clipped':False},{'route_endpoint_clipped':True},'gt_path')
    assert normalize_pair([gt[-1],model[-1]])[1]==1
    model,md=project(np.r_[np.linspace(0,12,21),np.linspace(11.4,6,10)])
    assert md['endpoint_clipped'] and normalize_pair([8,model[-1]])[1]==pytest.approx(.75)
    with pytest.raises(ValueError,match='GT baseline uncertain'):
        validate_endpoints({'route_endpoint_clipped':True},{'route_endpoint_clipped':False},'gt_path')
    with pytest.raises(ValueError,match='route too short'):
        validate_endpoints({'route_endpoint_clipped':False},{'route_endpoint_clipped':True},'centerline')
    assert normalize_pair([.2,0])[1]==0  # no low-progress relaxation


def records(tmp_path):
    result=[]
    for scene in ('b','a'):
        for i in range(7):
            folder=tmp_path/f'{scene}{i}';folder.mkdir()
            for name in ('scene.json','trajectories.npz'):(folder/name).touch()
            result.append(dict(token=f'{scene}{i}',scenario=scene,sample_start_time_s=100+i*.5,valid=True,_folder=folder))
    return list(reversed(result))


def test_period_scene_reset_order_and_missing_samples(tmp_path):
    rs=records(tmp_path)
    assert select_viewer_period(rs,0) is rs
    assert [r['token'] for r in select_viewer_period(rs,1.5)]==['a0','a3','a6','b0','b3','b6']
    assert [r['token'] for r in select_viewer_period(rs,1.)]==['a0','a2','a4','a6','b0','b2','b4','b6']
    next(r for r in rs if r['token']=='a3')['valid']=False
    assert [r['token'] for r in select_viewer_period(rs,1.5)][:2]==['a0','a4']
    next(r for r in rs if r['token']=='a4')['_folder'].joinpath('scene.json').unlink()
    assert [r['token'] for r in select_viewer_period(rs,1.5)][:2]==['a0','a5']


@pytest.mark.parametrize('period',[-1,float('nan'),float('inf')])
def test_bad_period(period):
    with pytest.raises(ValueError,match='period'):select_viewer_period([],period)


def test_old_results(tmp_path):
    rs=records(tmp_path);rs[0].pop('sample_start_time_s')
    assert select_viewer_period(rs,0)==rs
    with pytest.raises(ValueError,match='re-evaluate'):select_viewer_period(rs,1.5)


@pytest.mark.parametrize('gt_clipped',[False,True])
def test_evaluator_saves_valid_model_endpoint_and_invalid_gt(tmp_path,monkeypatch,gt_clipped):
    from test_cache_coordinates import common_data
    from etri_pdms.nuscenes_data import NuScenesDataset
    from etri_pdms.prediction import load_plans
    import etri_pdms.evaluator as ev
    pred=common_data(tmp_path);cfg=Config(dataset='nuscenes')
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',cfg)
    original=ev.score_pair
    def score(*args):
        gt,model=original(*args)
        gt['route_endpoint_clipped']=gt_clipped;model['route_endpoint_clipped']=True
        return gt,model
    monkeypatch.setattr(ev,'score_pair',score)
    out=tmp_path/'run';out.mkdir()
    row=evaluate_sample('s2',load_plans(pred),ds,out,cfg)
    assert row['valid']==(not gt_clipped)
    assert row['sample_start_time_s']==pytest.approx(1.)
    assert row['model_endpoint_exceeded'] and row['gt_endpoint_exceeded']==gt_clipped
    folder=out/'samples'/row['artifact_id']
    assert (folder/'diagnostics.json').is_file()
    assert (folder/'trajectories.npz').exists()==(not gt_clipped)
    if gt_clipped:assert 'GT baseline uncertain' in row['invalid_reason']


def test_cli_period_forwarding(monkeypatch,tmp_path):
    import sys
    import etri_pdms.cli as cli
    import etri_pdms.viewer as viewer
    seen={}
    def serve(run,host,port,period=0.):
        seen.update(run=run,port=port,period=period);return 0
    monkeypatch.setattr(viewer,'serve',serve)
    monkeypatch.setenv('PDMS_WS_ROOT',str(tmp_path))
    monkeypatch.setattr(sys,'argv',['pdms','serve','--run','example','--period','1.5'])
    assert cli.main()==0
    assert seen['period']==1.5 and seen['port']==7200
    assert seen['run']==str(tmp_path/'runs/example')
