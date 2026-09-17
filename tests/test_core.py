import numpy as np
import pytest
from shapely.geometry import LineString,box,Point
from etri_pdms.config import Config
from etri_pdms.prediction import select_prediction
from etri_pdms.geometry import center,ego_box,choose_route,local,rotation
from etri_pdms.metrics import normalize_pair,score_rollout,score_pair
from etri_pdms.tracking import reference,rollout

@pytest.fixture
def cfg(): return Config()

def straight(v=5):
    t=np.arange(31)*.1;states=np.column_stack([v*t,np.zeros(31),np.zeros(31),np.full(31,v)])
    return {'states':states,'controls':np.zeros((30,2))}

def scene():
    return {'route':LineString([[-30,0],[100,0]]),'lanes':[box(-30,-2,100,2)],'drivable':box(-30,-2,100,2),'intersection':box(100,100,101,101),'objects':[[] for _ in range(40)]}

def test_prediction_mode_and_no_double_cumsum(cfg):
    modes=np.zeros((3,6,2));modes[1,:,0]=2
    xy,idx=select_prediction([modes,np.array([[[[0,1,0]]]])],cfg)
    assert idx==1 and xy[-1,0]==12
    cfg.representation='relative_positions'
    assert select_prediction([modes,[0,1,0]],cfg)[0][-1,0]==2
    with pytest.raises(ValueError):select_prediction([modes,[.5,.5,0]],cfg)

def test_ep_reference_semantics():
    np.testing.assert_allclose(normalize_pair([20,10],[1,1],[1,1]),[1,.5])
    np.testing.assert_allclose(normalize_pair([20,10],[0,1],[1,1]),[1,.5])
    np.testing.assert_allclose(normalize_pair([0,0],[1,1],[1,1]),[1,1])
    np.testing.assert_allclose(normalize_pair([5,4],[1,1],[1,1]),[1,.8])
    np.testing.assert_allclose(normalize_pair([20,10],[.5,1],[1,1]),[1,.5])

def test_multi_id_progress(cfg):
    lines={'9':LineString([[-10,0],[10,0]]),'2':LineString([[10,0],[40,0]])}
    r,ids=choose_route(lines,straight()['states'],cfg)
    assert ids==['9','2']
    assert r.project(Point(15,0))-r.project(Point(0,0))==15

def test_vehicle_offset_and_coordinate_roundtrip(cfg):
    assert cfg.vehicle.center_offset==pytest.approx(1.5275)
    p=ego_box([0,0,0],cfg.vehicle)
    assert p.bounds==pytest.approx((-.790,-.946,3.845,.946))
    xy=np.array([[1,2],[3,4]]);pose=np.array([20,30,.5]);world=xy@rotation(pose[2]).T+pose[:2]
    np.testing.assert_allclose(local(world,pose),xy)

def test_collision_ttc_and_comfort(cfg):
    s=scene();obj={'id':'car','class':'Car','speed':0.,'xy':np.array([12,0]),'polygon':box(10,-1,14,1)}
    s['objects']=[[obj] for _ in range(40)];out=score_rollout(straight(),s,cfg)
    assert out['NC']==0 and out['TTC']==0 and out['C']==1
    assert min(e['time_s'] for e in out['events'] if e['metric']=='TTC') < min(e['time_s'] for e in out['events'] if e['metric']=='NC')

def test_rear_not_at_fault(cfg):
    s=scene();s['objects'][0]=[{'id':'rear','class':'Car','speed':10.,'xy':np.array([-2,0]),'polygon':box(-4,-1,0,1)}]
    out=score_rollout(straight(),s,cfg);assert out['NC']==1
    assert out['events'][0]['kind']=='rear'

def test_corner_offroad(cfg):
    roll=straight();roll['states'][:,1]=1.2
    assert score_rollout(roll,scene(),cfg)['DAC']==0

def test_equal_rollouts(cfg):
    a,b=score_pair(straight(),straight(),scene(),cfg)
    assert a['EP']==b['EP']==a['PDMS']==1

def test_straight_mpc(cfg):
    ref=reference(np.column_stack([np.arange(7)*2.5,np.zeros(7)]),np.arange(7)*.5)
    out=rollout(ref,[0,0,0,5],cfg)
    assert out['tracking_rmse']<.01
    assert np.max(np.abs(np.diff(out['controls'][:,0])))<=np.deg2rad(cfg.controller.steer_rate_deg)*.1+1e-5

def test_stop_preserves_measured_speed(cfg):
    ref=reference(np.zeros((7,2)),np.arange(7)*.5);out=rollout(ref,[0,0,0,5],cfg)
    assert out['states'][1,0]==pytest.approx(.5)
    assert out['states'][1,3]>=4.599
    assert np.min(out['states'][:,3])>=0


def test_gt_only_baseline_and_cap():
    np.testing.assert_allclose(normalize_pair([20,25],[1,1],[1,1]),[1,1])
    np.testing.assert_allclose(normalize_pair([20,10],[0,0],[0,0]),[1,.5])
    np.testing.assert_allclose(normalize_pair([0,10]),[1,0])
    np.testing.assert_allclose(normalize_pair([0,0]),[1,1])
