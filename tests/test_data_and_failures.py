import json
import numpy as np
import pandas as pd
import pytest
from etri_pdms.config import Config
from etri_pdms.data import Dataset,resample
from etri_pdms.demo import create_demo
from etri_pdms.evaluator import evaluate
from etri_pdms.tracking import TrackingFailure

def test_timestamp_gap_rejected():
    d=pd.DataFrame({'timestamp':[0,100,500],'x':[0,1,5]})
    with pytest.raises(ValueError,match='gap'):resample(d,np.array([.2]),['x'],Config())

def test_future_coverage_rejected():
    d=pd.DataFrame({'timestamp':[0,100,200],'x':[0,1,2]})
    with pytest.raises(ValueError,match='coverage'):resample(d,np.array([.3]),['x'],Config())

def test_unreviewed_polygon_is_not_validated(tmp_path):
    pred,infos,root=create_demo(tmp_path)
    path=root/'synthetic_straight'/'map_polygons.geojson';doc=json.loads(path.read_text());doc['properties']['verified']=False;path.write_text(json.dumps(doc))
    cfg=Config();cfg.map_mode='validated';ds=Dataset(root,infos,cfg)
    with pytest.raises(ValueError,match='verified'):ds.sample(next(iter(ds.infos)))

def test_mpc_failure_not_dropped(tmp_path,monkeypatch):
    pred,infos,root=create_demo(tmp_path);out=tmp_path/'evaluation'
    def fail(*a,**kw):raise TrackingFailure('deliberate solver failure')
    monkeypatch.setattr('etri_pdms.evaluator.rollout',fail)
    summary=evaluate(pred,infos,root,out,Config(),limit=1,visualize=0)
    assert summary['invalid']==1 and summary['requested']==1 and summary['complete'] is False
    rows=pd.read_csv(out/'scores.csv');assert len(rows)==1
    assert 'deliberate solver failure' in rows.iloc[0].invalid_reason
    assert 'scenario_macro_mean' not in summary

def test_missing_token_is_explicit(tmp_path):
    pred,infos,root=create_demo(tmp_path)
    summary=evaluate(pred,infos,root,tmp_path/'evaluation',Config(),tokens=['missing'],visualize=0)
    assert summary['invalid']==1 and summary['valid']==0
