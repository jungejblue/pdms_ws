import hashlib
import json
import pickle
import pytest
from etri_pdms.config import Config
from etri_pdms.data import Dataset
from etri_pdms.demo import create_demo
from etri_pdms.reporting import write_scenario_reports


def test_direct_original_files_unchanged(tmp_path):
    _,infos,root=create_demo(tmp_path)
    files=[infos,*root.rglob('*')]
    before={p:hashlib.sha256(p.read_bytes()).digest() for p in files if p.is_file()}
    ds=Dataset(root,infos,Config())
    sample=ds.sample(next(iter(ds.infos)))
    assert sample['gt'].shape==(31,3)
    assert sample['initial'][3]==pytest.approx(5)
    assert all(hashlib.sha256(p.read_bytes()).digest()==h for p,h in before.items())
    assert not (tmp_path/'raw_data.pkl').exists()


def test_meta_subdirectory(tmp_path):
    _,infos,root=create_demo(tmp_path)
    scene=root/'synthetic_straight';meta=scene/'meta';meta.mkdir()
    for path in scene.glob('*.parquet'):path.rename(meta/path.name)
    ds=Dataset(root,infos,Config())
    assert ds.sample(next(iter(ds.infos)))['scenario']=='synthetic_straight'


def test_requires_original_data_root(tmp_path):
    _,infos,_=create_demo(tmp_path)
    with pytest.raises(ValueError,match='--data-root'):Dataset(None,infos,Config())


def test_missing_scene_never_uses_other_scene(tmp_path):
    _,infos,root=create_demo(tmp_path)
    content=pickle.loads(infos.read_bytes())
    content['infos'].append({'token':'other_00000000','scene_token':'other','timestamp':1700000000000000})
    infos.write_bytes(pickle.dumps(content))
    ds=Dataset(root/'synthetic_straight',infos,Config())
    with pytest.raises(FileNotFoundError,match='Scenario directory missing'):ds.scene('other')


def test_scenario_json_mean_and_invalid_coverage(tmp_path):
    def row(scene,score):
        return dict(scenario=scene,valid=True,**{k:score for k in ('NC','DAC','EP','TTC','C','PDMS')})
    rows=[row('A',1),row('A',.5),row('B',.8),dict(scenario='B',valid=False,invalid_reason='missing future'),dict(scenario='C',valid=False,invalid_reason='solver failure')]
    result=write_scenario_reports(tmp_path,rows)
    assert result[0]['PDMS']==.75
    assert result[1]['PDMS'] is None and result[1]['coverage']==.5
    assert result[1]['valid_sample_mean']['PDMS']==.8
    assert result[2]['status']=='invalid' and result[2]['valid_sample_mean']['PDMS'] is None
    assert json.loads((tmp_path/'scenario_scores.json').read_text())==result
    assert len((tmp_path/'scenario_scores.json').read_text().splitlines())==5
    assert [json.loads(s) for s in (tmp_path/'scenario_scores.jsonl').read_text().splitlines()]==result
