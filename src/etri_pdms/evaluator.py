from pathlib import Path
import hashlib,json,time,traceback
import numpy as np
import pandas as pd
from .data import Dataset
from .reporting import write_scenario_reports
from . import __version__
from .prediction import load_plans,select_prediction
from .tracking import reference,rollout
from .metrics import score_pair
from .visualization import scene_payload,render_sample,render_png,index_report

METRICS=('PDMS','NC','DAC','EP','TTC','C')
def write_json(path,data):
    Path(path).write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')

def evaluate(pred_path,infos,root,out,cfg,limit=None,tokens=None,visualize=20):
    out=Path(out)
    if out.exists() and any(out.iterdir()): raise FileExistsError('Output already contains a run; choose a new --out directory')
    out.mkdir(parents=True,exist_ok=True);plans=load_plans(pred_path);dataset=Dataset(root,infos,cfg)
    requested=tokens if tokens is not None else list(plans)
    if limit is not None: requested=requested[:limit]
    if not requested: raise ValueError('No prediction tokens selected')
    if len(set(requested))!=len(requested): raise ValueError('Duplicate tokens in evaluation selection')
    (out/'evaluated_tokens.txt').write_text('\n'.join(map(str,requested))+'\n')
    rows=[];started=time.time()
    write_json(out/'config.resolved.json',cfg.to_dict())
    for index,token in enumerate(requested):
        row={'token':str(token),'valid':False,'invalid_reason':'','artifact_id':hashlib.sha256(str(token).encode()).hexdigest()[:20]}
        row.update({key:None for key in METRICS})
        row['scenario']=dataset.scenario_for(token) if token in dataset.infos else '__unmatched__'
        folder=out/'samples'/row['artifact_id'];folder.mkdir(parents=True,exist_ok=True)
        try:
            if token not in plans: raise ValueError('requested token missing in planning PKL')
            if token not in dataset.infos: raise ValueError('prediction token has no ETRI info match; nuScenes PKL is a format example only')
            sample=dataset.sample(token);row['scenario']=sample['scenario'];row['map_quality']=sample['map_quality'];row['route_source']=sample['route_source']
            pred,mode=select_prediction(plans[token],cfg)
            pred_ref=reference(np.vstack([[0,0],pred]),np.arange(7)*.5)
            gt_ref=reference(sample['gt'][:,:2],np.arange(31)*.1)
            gt_roll=rollout(gt_ref,sample['initial'],cfg);model_roll=rollout(pred_ref,sample['initial'],cfg)
            gt_score,model_score=score_pair(gt_roll,model_roll,sample,cfg)
            if gt_score['route_endpoint_clipped'] or model_score['route_endpoint_clipped']:
                raise ValueError('EP route too short; extend route_ids.json through the intended branch')
            row.update({k:model_score[k] for k in METRICS});row.update({'gt_'+k:gt_score[k] for k in METRICS})
            row.update(valid=True,command_index=mode,route_ids=sample['route_ids'],reference_failure=bool(gt_score['NC']*gt_score['DAC']==0),
                       reference_progress_m=gt_score['progress_m'],model_progress_m=model_score['progress_m'],
                       tracking_rmse=model_roll['tracking_rmse'],gt_tracking_rmse=gt_roll['tracking_rmse'],
                       raw_ADE=float(np.linalg.norm(pred-sample['gt'][5::5,:2],axis=1).mean()),
                       raw_FDE=float(np.linalg.norm(pred[-1]-sample['gt'][-1,:2])),
                       solver_success_rate=1.,max_clf_slack=float(model_roll['clf_slack'].max()),
                       route_endpoint_clipped=bool(model_score['route_endpoint_clipped'] or gt_score['route_endpoint_clipped']))
            np.savez_compressed(folder/'trajectories.npz',pred_raw=pred,pred_reference=pred_ref,gt_reference=gt_ref,
                                gt_raw=sample['gt'],pred_rollout=model_roll['states'],gt_rollout=gt_roll['states'],
                                pred_controls=model_roll['controls'],gt_controls=gt_roll['controls'],
                                pred_clf_slack=model_roll['clf_slack'],gt_clf_slack=gt_roll['clf_slack'])
            write_json(folder/'diagnostics.json',{'model':model_score,'gt':gt_score,'model_solver':model_roll['solver_status'],'gt_solver':gt_roll['solver_status']})
            write_json(folder/'scene.json',scene_payload(sample,cfg))
        except (ValueError,KeyError,OSError,RuntimeError,IndexError,TypeError) as exc:
            row['valid']=False
            row.update({key:None for key in METRICS})
            row['invalid_reason']=f'{type(exc).__name__}: {exc}'
            (folder/'error.txt').write_text(traceback.format_exc())
        write_json(folder/'result.json',row);rows.append(row)
        print(f'[{index+1}/{len(requested)}] {token}: '+(f'PDMS={row["PDMS"]:.4f}' if row['valid'] else row['invalid_reason']),flush=True)
    valid=[r for r in rows if r['valid']];frame=pd.DataFrame(rows);frame.to_csv(out/'scores.csv',index=False)
    summary={'metric':'ETRI-PDMS-GT-MPC-v0.1','package_version':__version__,'config_hash':cfg.hash(),'requested':len(rows),'valid':len(valid),'invalid':len(rows)-len(valid),
             'complete':len(valid)==len(rows),'map_mode':cfg.map_mode,'official_navsim_comparable':False,
             'reference_failure_count':sum(r['reference_failure'] for r in valid),'elapsed_s':round(time.time()-started,2)}
    if valid:
        df=pd.DataFrame(valid);macro=df.groupby('scenario')[list(METRICS)].mean()
        summary['valid_sample_scenario_macro_mean']=macro.mean().to_dict();summary['valid_sample_micro_mean']=df[list(METRICS)].mean().to_dict()
        summary['scenario_macro_mean']=macro.mean().to_dict() if summary['complete'] else None
        summary['failure_rates']={k:float((df[k]<1).mean()) for k in ('NC','DAC','TTC','C')}
        summary['PDMS_quantiles']={str(k):float(df.PDMS.quantile(k)) for k in (.01,.05,.1,.5)}
        summary['map_quality_counts']=df.map_quality.value_counts().to_dict()
    if not summary['complete']: summary['aggregation_warning']='Means use valid rows only; incomplete run is not a benchmark result. All invalid tokens remain in scores.csv.'
    scenario_rows=write_scenario_reports(out,rows)
    summary['scenarios']=len(scenario_rows)
    summary['scenario_output']='scenario_scores.json'
    summary['visualization_errors']=0
    write_json(out/'sample_scores.json',rows)
    write_json(out/'summary.json',summary)
    # localhost reads NPZ/scene.json directly. Optional HTML export cannot erase scores.
    for row in sorted([r for r in rows if r['valid']],key=lambda r:r['PDMS'])[:visualize]:
        folder=out/'samples'/row['artifact_id']
        try:
            render_sample(folder);render_png(folder);row['visualized']=True
        except Exception as exc:
            row['visualization_error']=f'{type(exc).__name__}: {exc}'
            summary['visualization_errors']+=1
            (folder/'visualization_error.txt').write_text(traceback.format_exc())
        write_json(folder/'result.json',row)
    pd.DataFrame(rows).to_csv(out/'scores.csv',index=False)
    write_json(out/'sample_scores.json',rows)
    write_json(out/'summary.json',summary);index_report(out,rows,summary)
    return summary
