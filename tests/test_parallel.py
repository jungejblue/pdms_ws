import json,pickle
import numpy as np
import pytest
from pathlib import Path
from etri_pdms.initial_speed import initial_speed
from etri_pdms.nuscenes_data import interpolate
from etri_pdms.evaluator import evaluate
from etri_pdms.config import Config
from test_cache_coordinates import common_data


def test_start_and_backward_speed():
    ts=np.arange(10)*.05;xy=np.c_[ts*8,np.zeros(len(ts))]
    sample=lambda q:interpolate(ts,xy,q,.16)
    v,meta=initial_speed(0,0,ts[-1],sample)
    assert v==pytest.approx(8) and meta['initial_speed_uses_future']
    v,meta=initial_speed(.2,0,ts[-1],sample)
    assert v==pytest.approx(8) and not meta['initial_speed_uses_future']
    v,meta=initial_speed(.05,0,ts[-1],sample)
    assert v==pytest.approx(8) and meta['initial_speed_uses_future']


def test_no_fallback_for_internal_gaps_or_missing_future():
    ts=np.array([0,.5,1]);xy=np.c_[ts,np.zeros(3)]
    f=lambda q:interpolate(ts,xy,q,.16)
    with pytest.raises(ValueError,match='gap'):initial_speed(.5,0,1,f)
    with pytest.raises(ValueError,match='coverage'):initial_speed(0,0,.05,lambda q:interpolate([0,.05],[[0,0],[1,0]],q,.16))


@pytest.mark.parametrize('workers',[0,5,-1,True,1.5])
def test_worker_limit(workers,tmp_path):
    with pytest.raises(ValueError,match='workers'):evaluate(None,None,None,tmp_path/'out',Config(),workers=workers)
    assert not (tmp_path/'out').exists()


def two_scenes(root):
    pred=common_data(root);meta=root/'v1.0-mini'
    docs={p.name:json.loads(p.read_text()) for p in meta.glob('*.json')}
    tokenmap={r['token']:'b_'+r['token'] for rows in docs.values() for r in rows}
    def rename(v):
        if isinstance(v,str):return tokenmap.get(v,v)
        if isinstance(v,list):return [rename(x) for x in v]
        if isinstance(v,dict):return {k:rename(x) for k,x in v.items()}
        return v
    for name,rows in docs.items():(meta/name).write_text(json.dumps(rows+rename(rows)))
    with (root/'infos.pkl').open('rb') as f:doc=pickle.load(f)
    doc['infos']+=rename(doc['infos'])
    with (root/'infos.pkl').open('wb') as f:pickle.dump(doc,f)
    keys=['s0','b_s2','s14','b_s0','s2','missing']
    with pred.open('wb') as f:pickle.dump({'plan_results':{k:[np.tile([2.5,0],(6,1)),0] for k in keys}},f)
    return pred,keys


def test_parallel_equals_serial_with_start_and_invalid(tmp_path):
    pred,keys=two_scenes(tmp_path)
    cfg=Config(dataset='nuscenes')
    a=evaluate(pred,tmp_path/'infos.pkl',tmp_path,tmp_path/'serial',cfg,visualize=0,workers=1)
    b=evaluate(pred,tmp_path/'infos.pkl',tmp_path,tmp_path/'parallel',cfg,visualize=0,workers=2)
    assert a['valid']==b['valid']==4
    assert a['invalid']==b['invalid']==2
    assert b['workers_effective']==2 and b['forward_initial_speed_count']==2
    left=json.loads((tmp_path/'serial/sample_scores.json').read_text())
    right=json.loads((tmp_path/'parallel/sample_scores.json').read_text())
    assert [r['token'] for r in right]==keys
    assert left==right
    for row in left:
        if not row['valid']:continue
        path=Path('samples')/row['artifact_id']/'trajectories.npz'
        with np.load(tmp_path/'serial'/path) as x,np.load(tmp_path/'parallel'/path) as y:
            for key in x.files:assert np.allclose(x[key],y[key],atol=1e-10,rtol=1e-10)
    manifest=json.loads((tmp_path/'parallel/input_manifest.json').read_text())
    assert manifest['infos']['map_files']


def test_etri_parallel(tmp_path):
    from etri_pdms.demo import create_demo
    pred,infos,root=create_demo(tmp_path/'data')
    cfg=Config(map_mode='validated')
    a=evaluate(pred,infos,root,tmp_path/'a',cfg,visualize=0,workers=1)
    b=evaluate(pred,infos,root,tmp_path/'b',cfg,visualize=0,workers=4)
    assert a['valid']==b['valid']==3
    assert json.loads((tmp_path/'a/sample_scores.json').read_text())==json.loads((tmp_path/'b/sample_scores.json').read_text())
