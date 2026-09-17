"""Preselect fixed-horizon samples from raw recording bounds, never from scores.

Only insufficient remaining duration is excluded. Missing/corrupt inputs and
interior gaps remain evaluation errors, not silently accepted exclusions.
"""
import numpy as np
import pandas as pd
from .data import seconds


def _end(values):
    values=np.asarray(values,dtype=float)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError('Missing/nonfinite recording timestamps')
    return float(values.max())


def _etri_ends(dataset,scene):
    folder=dataset.root/scene
    if not folder.is_dir():
        if len(dataset.scene_names)==1:folder=dataset.root
        else:raise FileNotFoundError(f'Scenario directory missing: {folder}')
    def read(name,columns):
        for path in (folder/name,folder/'meta'/name):
            if path.is_file():return pd.read_parquet(path,columns=columns)
        raise FileNotFoundError(f'Missing {name}')
    ego=read('ego_pose.parquet',['timestamp'])
    objects=read('object.parquet',['timestamp','class'])
    objects=objects[objects['class'].str.lower()=='ego']
    unit=dataset.cfg.raw_timestamp_unit
    return _end(seconds(ego.timestamp,unit)),_end(seconds(objects.timestamp,unit))


def select_coverage(tokens,dataset):
    if len(set(tokens))!=len(tokens):raise ValueError('Duplicate tokens in evaluation selection')
    kept=[];excluded=[];errors=[];bounds={}
    for token in tokens:
        if token not in dataset.infos:
            kept.append(token);continue
        try:
            scene=dataset.scenario_for(token)
            if dataset.cfg.dataset=='nuscenes':
                key=dataset.lidar_key[token]
                t0=float(key['timestamp'])/1e6
                if scene not in bounds:
                    bounds[scene]=(_end([s['timestamp']/1e6 for s in dataset.scene_lidar[scene]]),
                                   _end([s['timestamp']/1e6 for s in dataset.scene_samples[scene]]))
                tolerance=1e-6
            else:
                t0=float(seconds(dataset.infos[token]['timestamp'],dataset.cfg.info_timestamp_unit))
                if scene not in bounds:bounds[scene]=_etri_ends(dataset,scene)
                tolerance=1e-5
            if not np.isfinite(t0):raise ValueError('Nonfinite sample timestamp')
            gt_end,obj_end=bounds[scene]
            reasons=[]
            if t0+3.0>gt_end+tolerance:reasons.append('insufficient_gt_future')
            if t0+3.9>obj_end+tolerance:reasons.append('insufficient_object_future')
            if reasons:
                excluded.append(dict(token=token,scenario=scene,reasons=reasons,
                                     gt_remaining_s=gt_end-t0,object_remaining_s=obj_end-t0))
            else:kept.append(token)
        except (ValueError,KeyError,OSError,TypeError,AttributeError) as exc:
            # Preserve bad metadata as an invalid evaluation, not an exclusion.
            kept.append(token)
            errors.append(dict(token=token,error=f'{type(exc).__name__}: {exc}'))
    return kept,dict(policy='fixed_horizon_remaining_duration',gt_required_s=3.0,
                     objects_required_s=3.9,input_count=len(tokens),eligible_count=len(kept),
                     excluded_count=len(excluded),excluded_samples=excluded,precheck_errors=errors)
