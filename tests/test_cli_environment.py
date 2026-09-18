import sys
from pathlib import Path
import pytest
from etri_pdms import cli


def test_run_paths(tmp_path, monkeypatch):
    monkeypatch.setenv('PDMS_WS_ROOT', str(tmp_path))
    assert cli.resolve_run_path('demo') == tmp_path/'runs/demo'
    assert cli.resolve_run_path('runs/demo') == tmp_path/'runs/demo'
    assert cli.resolve_run_path(str(tmp_path/'outside')) == tmp_path/'outside'
    with pytest.raises(ValueError): cli.resolve_run_path('../outside')


def test_short_evaluate_env_and_override(tmp_path, monkeypatch):
    from etri_pdms import evaluator
    monkeypatch.setenv('PDMS_WS_ROOT', str(tmp_path))
    (tmp_path/'configs').mkdir()
    (tmp_path/'configs/ioniq5_2023.yaml').write_text('{}')
    for attr in ('PDMS_RAW_PKL','PDMS_PLANNING_PKL'):
        p=tmp_path/attr;p.touch();monkeypatch.setenv(attr,str(p))
    monkeypatch.setenv('PDMS_DATA_ROOT',str(tmp_path))
    seen=[]
    monkeypatch.setattr(evaluator,'evaluate',lambda *a: seen.append(a) or {'complete':True})
    monkeypatch.setattr(sys,'argv',['pdms','evaluate','--out','test'])
    assert cli.main()==0
    assert seen[-1][3]==str(tmp_path/'runs/test')
    other=tmp_path/'other.pkl';other.touch()
    monkeypatch.setattr(sys,'argv',['pdms','evaluate','--out','runs/other','--planning-pkl',str(other)])
    assert cli.main()==0 and seen[-1][0]==str(other)


def test_serve_defaults_and_docker(tmp_path, monkeypatch):
    from etri_pdms import viewer
    monkeypatch.setenv('PDMS_WS_ROOT',str(tmp_path))
    monkeypatch.delenv('PDMS_HOST',raising=False)
    seen=[]
    monkeypatch.setattr(viewer,'serve',lambda *a,period=0.: seen.append((*a,period)) or 0)
    monkeypatch.setattr(sys,'argv',['pdms','serve','--run','test'])
    cli.main()
    assert seen[-1]==(str(tmp_path/'runs/test'),'127.0.0.1',7200,0.)
    monkeypatch.setenv('PDMS_HOST','0.0.0.0');cli.main()
    assert seen[-1][1]=='0.0.0.0'


def test_missing_input_is_actionable(monkeypatch):
    monkeypatch.delenv('PDMS_PLANNING_PKL',raising=False)
    monkeypatch.setattr(sys,'argv',['pdms','evaluate','--out','test'])
    with pytest.raises(SystemExit) as exc: cli.main()
    assert exc.value.code==2
