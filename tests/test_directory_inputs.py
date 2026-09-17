import json
import pickle
import numpy as np
import pytest
from etri_pdms.inputs import load_infos
from etri_pdms.prediction import load_plans
from etri_pdms.demo import create_demo
from etri_pdms.evaluator import evaluate
from etri_pdms.config import Config


def put(path,obj): path.write_bytes(pickle.dumps(obj))
def entry(x=0): return [np.full((6,2),float(x)),[0,0,1]]


def test_merge_dedup_and_skip(tmp_path):
    put(tmp_path/'odd.pkl',{'plan_results':{'a':entry()}})
    put(tmp_path/'other.pickle',{'plan_results':{'a':entry(),'b':entry(1)}})
    put(tmp_path/'irrelevant.pkl',{'other':[]})
    nested=tmp_path/'old';nested.mkdir();put(nested/'ignore.pkl',{'plan_results':{'c':entry()}})
    plans,report=load_plans(tmp_path,return_report=True)
    assert set(plans)=={'a','b'} and report['duplicate_tokens']==1
    assert len(report['selected_files'])==2 and len(report['skipped_files'])==1


def test_prediction_conflict(tmp_path):
    for name,x in [('a',0),('b',1)]:put(tmp_path/(name+'.pkl'),{'plan_results':{'token':entry(x)}})
    with pytest.raises(ValueError,match='Conflicting planning token'):load_plans(tmp_path)


def test_infos_merge_and_conflict(tmp_path):
    i={'token':'s_0','scene_token':'s','timestamp':100}
    put(tmp_path/'a.pkl',{'infos':[i]});put(tmp_path/'b.pkl',[dict(i,unused=3)])
    infos,r=load_infos(tmp_path);assert len(infos)==1 and r['duplicate_tokens']==1
    put(tmp_path/'b.pkl',[dict(i,timestamp=101)])
    with pytest.raises(ValueError,match='Conflicting infos token'):load_infos(tmp_path)


def test_corrupt_not_silently_skipped(tmp_path):
    (tmp_path/'broken.pkl').write_bytes(b'broken')
    with pytest.raises(ValueError,match='Cannot read PKL'):load_plans(tmp_path)


def test_empty_or_wrong_schema(tmp_path):
    with pytest.raises(ValueError,match='No usable'):load_plans(tmp_path)
    put(tmp_path/'wrong.pkl',{'hello':1})
    with pytest.raises(ValueError,match='not a planning'):load_plans(tmp_path/'wrong.pkl')


def test_malformed_recognized_schema(tmp_path):
    put(tmp_path/'bad.pkl',{'plan_results':[]})
    with pytest.raises(ValueError,match='must be a dictionary'):load_plans(tmp_path)


def test_directory_end_to_end_and_manifest(tmp_path):
    pred,infos,data=create_demo(tmp_path/'demo')
    plan_dir=tmp_path/'plans';info_dir=tmp_path/'infos';plan_dir.mkdir();info_dir.mkdir()
    plans=pickle.loads(pred.read_bytes())['plan_results']
    for n,(token,value) in enumerate(plans.items()):put(plan_dir/f'arbitrary{n}.pkl',{'plan_results':{token:value}})
    put(info_dir/'arbitrary.pkl',pickle.loads(infos.read_bytes()))
    summary=evaluate(plan_dir,info_dir,data,tmp_path/'out',Config(),visualize=0)
    assert summary['complete'] and summary['valid']==3
    manifest=json.loads((tmp_path/'out/input_manifest.json').read_text())
    assert len(manifest['planning']['selected_files'])==3
    assert len(manifest['planning']['token_sources'])==3
