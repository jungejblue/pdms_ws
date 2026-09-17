import json,pickle
from types import SimpleNamespace
import numpy as np
import pandas as pd
import pytest
from etri_pdms.coverage import select_coverage
from etri_pdms.config import Config
from etri_pdms.nuscenes_data import NuScenesDataset
from etri_pdms.evaluator import evaluate
from test_nuscenes import make_data


def test_nuscenes_all_samples_and_boundaries(tmp_path):
    make_data(tmp_path)
    ds=NuScenesDataset(tmp_path,tmp_path/'infos.pkl',Config(dataset='nuscenes',nuscenes_prediction_frame='lidar'))
    kept,report=select_coverage(list(ds.infos),ds)
    assert kept==[f's{i}' for i in range(7)]
    assert report['excluded_count']==8
    assert report['excluded_samples'][0]['reasons']==['insufficient_object_future']
    assert not ds.maps and not ds.cache
    ds.lidar_key['s6']['timestamp']=3_100_000
    assert select_coverage(['s6'],ds)[0]==['s6']
    ds.lidar_key['s6']['timestamp']=3_100_100
    assert select_coverage(['s6'],ds)[0]==[]


def test_etri_units_and_missing_input(tmp_path):
    folder=tmp_path/'scene'/'meta';folder.mkdir(parents=True)
    pd.DataFrame({'timestamp':np.arange(0,5001,100)}).to_parquet(folder/'ego_pose.parquet')
    pd.DataFrame({'timestamp':np.arange(0,5001,100),'class':'ego'}).to_parquet(folder/'object.parquet')
    ds=SimpleNamespace(root=tmp_path,cfg=Config(),scene_names={'scene'},
        infos={k:{'timestamp':t} for k,t in [('start',0),('edge',1100000),('late',1200000)]},
        scenario_for=lambda token:'scene')
    kept,report=select_coverage(['start','edge','late','missing'],ds)
    assert kept==['start','edge','missing']
    assert report['excluded_samples'][0]['reasons']==['insufficient_object_future']
    (folder/'object.parquet').unlink()
    assert select_coverage(['start'],ds)[1]['precheck_errors']
    assert select_coverage(['start'],ds)[0]==['start']


def test_limit_after_exclusion_and_all_excluded(tmp_path):
    pred=make_data(tmp_path)
    cfg=Config(dataset='nuscenes',nuscenes_prediction_frame='lidar')
    def plans(keys):
        with pred.open('wb') as f:pickle.dump({'plan_results':{k:[np.tile([0.,-2.5],(6,1)),0] for k in keys}},f)
    plans(['s14','s2','s3']);out=tmp_path/'run'
    summary=evaluate(pred,tmp_path/'infos.pkl',tmp_path,out,cfg,limit=1,visualize=0)
    assert summary['requested']==1 and summary['time_excluded']==1 and summary['valid']==1
    assert (out/'evaluated_tokens.txt').read_text().splitlines()==['s2']
    assert len(list((out/'samples').iterdir()))==1
    plans(['s14'])
    with pytest.raises(ValueError,match='No time-eligible'):
        evaluate(pred,tmp_path/'infos.pkl',tmp_path,tmp_path/'empty',cfg,visualize=0,workers=4)
    assert json.loads((tmp_path/'empty/excluded_samples.json').read_text())['excluded_count']==1
    assert not (tmp_path/'empty/samples').exists()
