import pickle,json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from etri_pdms.cache_coordinates import cache_pose,EXPECTED
from etri_pdms.nuscenes_data import NuScenesDataset
from etri_pdms.config import Config
from etri_pdms.inputs import load_infos
from test_nuscenes import make_data


def common_data(root):
    pred=make_data(root)
    path=root/'infos.pkl'
    with path.open('rb') as f:doc=pickle.load(f)
    for info in doc['infos']:
        info.update(conversion_meta={**EXPECTED,'quaternion_order':'wxyz'},
                    ego2global_translation=[5*info['timestamp']/1e6,0,0],
                    ego2global_rotation=[1,0,0,0])
    with path.open('wb') as f:pickle.dump(doc,f)
    with pred.open('wb') as f:pickle.dump({'plan_results':{'s2':[np.tile([2.5,0.],(6,1)),0]}},f)
    return pred


def test_no_lidar_rotation_and_rear_axle_origin(tmp_path):
    common_data(tmp_path)
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes'))
    s=ds.sample('s2')
    assert np.allclose(ds.transform_prediction(np.array([[15.,0.]]),s),[[15,0]])
    assert np.allclose(s['gt'][-1,:2],[15,0])
    assert np.allclose(s['evaluation_origin'],[5,0,0]) # not LiDAR's +1 m
    assert s['objects'][0][0]['polygon'].bounds==pytest.approx((-2,19,2,21))
    assert ds.input_report['vehicle_anchor']=='cache_rear_axle_center'


def test_current_cache_end_to_end(tmp_path):
    from etri_pdms.evaluator import evaluate
    pred=common_data(tmp_path)
    result=evaluate(pred,tmp_path/'infos.pkl',tmp_path,tmp_path/'out',Config(dataset='nuscenes'),visualize=0)
    assert result['valid']==1
    row=json.loads((tmp_path/'out/sample_scores.json').read_text())[0]
    assert row['raw_ADE']<1e-8 and row['PDMS']>.99
    assert row['coordinate_diagnostics']['pose_position_error_m']==0


@pytest.mark.parametrize('field,bad',[('coordinate_axes','x_right_y_forward_z_up'),('ego_origin','lidar'),('coordinate_frame','lidar'),('quaternion_order','xyzw')])
def test_reject_wrong_metadata(tmp_path,field,bad):
    common_data(tmp_path)
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes'))
    ds.infos['s2']['conversion_meta'][field]=bad
    with pytest.raises(ValueError):ds.sample('s2')


def test_reject_missing_metadata_and_pose_mismatch(tmp_path):
    common_data(tmp_path)
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes'))
    ds.infos['s2']['map_ego2global_translation']=[8,0,0]
    with pytest.raises(ValueError,match='pose mismatch'):ds.check_coordinates('s2')
    ds.infos['s2'].pop('conversion_meta')
    with pytest.raises(ValueError,match='Missing conversion_meta'):ds.check_coordinates('s2')


def test_metadata_preserved_and_legacy_override_rejected(tmp_path):
    common_data(tmp_path)
    rows,_=load_infos(tmp_path/'infos.pkl')
    assert rows[0]['conversion_meta']['ego_origin']=='rear_axle_center'
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes',nuscenes_prediction_frame='lidar'))
    with pytest.raises(ValueError,match='use --prediction-frame cache'):ds.sample('s2')


def test_yaw_and_tilt_projection_matches_global_geometry(tmp_path):
    common_data(tmp_path)
    r=Rotation.from_euler('zyx',[45,4,2],degrees=True).as_matrix()
    info={'conversion_meta':EXPECTED,'ego2global_rotation':r.flatten(),'ego2global_translation':[100,50,2]}
    cr,t=cache_pose(info)
    assert np.allclose(cr,r)
    heading=np.arctan2(r[1,0],r[0,0]);o=np.array([100,50,heading])
    pred=np.array([[1,0],[4,1],[10,-2]],float)
    s={'cache_rotation':cr,'cache_translation':t,'evaluation_origin':o}
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes'))
    local=ds.transform_prediction(pred,s)
    rz=np.array([[np.cos(heading),-np.sin(heading)],[np.sin(heading),np.cos(heading)]])
    assert np.allclose(local@rz.T+o[:2],(np.c_[pred,np.zeros(len(pred))]@r.T+t)[:,:2])


def test_map_pose_priority_and_quaternion():
    q=Rotation.from_euler('z',40,degrees=True).as_quat()
    info={'conversion_meta':EXPECTED,'ego2global_rotation':[1,0,0,0],'ego2global_translation':[0,0,0],
          'map_ego2global_rotation':q[[3,0,1,2]],'map_ego2global_translation':[1,2,3]}
    r,t=cache_pose(info)
    assert np.allclose(r,Rotation.from_quat(q).as_matrix())
    assert np.allclose(t,[1,2,3])
