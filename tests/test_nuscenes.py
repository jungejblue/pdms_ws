import json,pickle,sys
import numpy as np
import pytest
from etri_pdms.config import Config
from etri_pdms.nuscenes_data import NuScenesDataset,interpolate
from etri_pdms.nuscenes_map import discretize_arcline


def make_data(root):
    tables={k:[] for k in ('sample','scene','sample_data','ego_pose','calibrated_sensor','sensor','log','sample_annotation','instance','category')}
    tables['scene']=[dict(token='scene',name='scene-0001',log_token='log')]
    tables['log']=[dict(token='log',location='test-map')]
    tables['sensor']=[dict(token='lidar',channel='LIDAR_TOP')]
    # 90-degree calibrated sensor rotation verifies actual LiDAR -> ego transform.
    tables['calibrated_sensor']=[dict(token='cal',sensor_token='lidar',translation=[1,0,2],rotation=[2**-.5,0,0,2**-.5])]
    tables['category']=[dict(token='car',name='vehicle.car')]
    tables['instance']=[dict(token='agent',category_token='car')]
    for i in range(15):
        token=f's{i}';t=i*.5
        tables['sample'].append(dict(token=token,scene_token='scene',timestamp=int(t*1e6)))
        tables['sample_annotation'].append(dict(token=f'a{i}',sample_token=token,instance_token='agent',translation=[t*5,20,0],size=[2,4,1.5],rotation=[1,0,0,0]))
    for i in range(141):
        t=i*.05;token=f'p{i}'
        tables['ego_pose'].append(dict(token=token,translation=[5*t,0,0],rotation=[1,0,0,0]))
        tables['sample_data'].append(dict(token=f'd{i}',timestamp=int(round(t*1e6)),sample_token=f's{min((i+9)//10,14)}',is_key_frame=i%10==0,ego_pose_token=token,calibrated_sensor_token='cal'))
    meta=root/'v1.0-mini';meta.mkdir(parents=True)
    for k,v in tables.items():(meta/(k+'.json')).write_text(json.dumps(v))
    points=[(-20,-5),(150,-5),(150,5),(-20,5)]
    doc=dict(node=[dict(token=str(i),x=x,y=y) for i,(x,y) in enumerate(points)],
             polygon=[dict(token='polygon',exterior_node_tokens=list(map(str,range(4))),holes=[])],
             drivable_area=[dict(token='drive',polygon_tokens=['polygon'])],lane=[dict(token='lane',polygon_token='polygon')],
             lane_connector=[],road_segment=[],connectivity={'lane':dict(outgoing=[],incoming=[])},
             arcline_path_3={'lane':[dict(start_pose=[-20,0,0],end_pose=[150,0,0],radius=10,shape='LSL',segment_length=[0,170,0])]})
    maps=root/'maps/expansion';maps.mkdir(parents=True)
    (maps/'test-map.json').write_text(json.dumps(doc))
    pred=root/'prediction.pkl'
    with pred.open('wb') as f:pickle.dump({'plan_results':{'s2':[np.tile([0.,-2.5],(6,1)),0]}},f)
    with (root/'infos.pkl').open('wb') as f:pickle.dump({'infos':[{'token':r['token'],'timestamp':r['timestamp']} for r in tables['sample']]},f)
    return pred


def test_coordinate_map_objects_and_coverage(tmp_path):
    make_data(tmp_path);cfg=Config(dataset='nuscenes')
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',cfg);s=ds.sample('s2')
    assert np.allclose(s['gt'][-1,:2],[15,0])
    assert np.allclose(ds.transform_prediction(np.array([[0,-15.]]),s),[[15,0]])
    assert s['objects'][0][0]['polygon'].bounds==pytest.approx((-3,19,1,21))
    assert s['map_quality']=='nuscenes_map_expansion'
    assert ds.input_report['map_files'] and ds.input_report['infos_pkl_required']
    with pytest.raises(ValueError,match='coverage'):ds.sample('s14')
    with pytest.raises(ValueError,match='coverage'):ds.sample('s0')


def test_arc_and_interpolation():
    p=discretize_arcline([dict(start_pose=[0,0,0],radius=10,shape='LSL',segment_length=[5*np.pi,0,0])])
    assert p[-1]==pytest.approx([10,10])
    vals=np.array([[0,np.deg2rad(179)],[1,np.deg2rad(-179)]])
    assert interpolate([0,.5],vals,[.25],.75,(1,))[0,1]==pytest.approx(np.pi)
    with pytest.raises(ValueError,match='gap'):interpolate([0,1],vals,[.5],.75)
    with pytest.raises(ValueError,match='coverage'):interpolate([0,.5],vals,[.6],.75)


def test_evaluate_and_saved_viewer_payload(tmp_path):
    from etri_pdms.evaluator import evaluate
    pred=make_data(tmp_path);out=tmp_path/'run'
    cfg=Config(dataset='nuscenes')
    summary=evaluate(pred,tmp_path/'infos.pkl',tmp_path,out,cfg,visualize=0)
    assert summary['valid']==1 and summary['complete']
    rows=json.loads((out/'sample_scores.json').read_text())
    assert rows[0]['raw_ADE']<1e-8 and rows[0]['PDMS']>.95
    assert rows[0]['anchor_assumption']=='virtual_rear_axle_at_lidar_origin'
    scene=json.loads((out/'samples'/rows[0]['artifact_id']/'scene.json').read_text())
    assert len(scene['objects'])==31
    from etri_pdms.viewer import discover_samples,load_sample
    assert load_sample(discover_samples(out)[0])['arrays']['pred_rollout'].shape==(31,4)
    assert len(json.loads((out/'scenario_scores.json').read_text()))==1


def test_nuscenes_short_cli(tmp_path,monkeypatch,capsys):
    from etri_pdms import cli
    pred=make_data(tmp_path)
    monkeypatch.setenv('PDMS_DATASET','nuscenes')
    monkeypatch.setenv('NUSCENES_DATA_ROOT',str(tmp_path))
    monkeypatch.setenv('NUSCENES_CACHE_PATH',str(tmp_path/'infos.pkl'))
    monkeypatch.setenv('NUSCENES_PREDICTION_CACHE',str(pred))
    monkeypatch.setenv('ETRI_CACHE_PATH','/irrelevant/etri')
    monkeypatch.setattr(sys,'argv',['pdms','inspect'])
    assert cli.main()==0
    result=json.loads(capsys.readouterr().out)
    assert result['matching_dataset_tokens']==1
    assert result['raw_tokens_without_predictions']==14


def test_infos_limits_selection_and_timestamp(tmp_path):
    make_data(tmp_path)
    path=tmp_path/'subset.pkl'
    with path.open('wb') as f:pickle.dump({'infos':[dict(token='s2',timestamp=1_000_000)]},f)
    ds=NuScenesDataset(tmp_path,path,Config(dataset='nuscenes'))
    assert set(ds.infos)=={'s2'}
    with pytest.raises(ValueError,match='infos PKL match'):ds.sample('s3')
    ds.infos['s2']['timestamp']=2_000_000
    with pytest.raises(ValueError,match='timestamp'):ds.sample('s2')


def test_missing_map_and_unknown_raw_token(tmp_path):
    make_data(tmp_path)
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes'))
    ds.infos['unknown']=dict(token='unknown',timestamp=1_000_000)
    assert ds.scenario_for('unknown')=='__unmatched__'
    with pytest.raises(ValueError,match='raw sample match'):ds.sample('unknown')
    (tmp_path/'maps/expansion/test-map.json').unlink()
    with pytest.raises(FileNotFoundError,match='map expansion'):ds.sample('s2')
