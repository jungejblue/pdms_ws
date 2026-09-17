"""Bounded scene jobs; each process owns its solver and scene data."""
from collections import OrderedDict
from concurrent.futures import ProcessPoolExecutor,wait,FIRST_COMPLETED
from copy import copy
import multiprocessing as mp
import os

_WORKER_MAPS={}

def scene_dataset(dataset,scene,tokens):
    d=copy(dataset);d.cache=OrderedDict()
    d.infos={t:dataset.infos[t] for t in tokens if t in dataset.infos}
    d.input_report={'map_files':[], 'vehicle_anchor':dataset.input_report.get('vehicle_anchor')}
    if dataset.cfg.dataset!='nuscenes':return d
    d.maps={}
    samples=dataset.scene_samples.get(scene,[])
    lidar=dataset.scene_lidar.get(scene,[])
    annotations=dataset.scene_annotations.get(scene,[])
    d.scene_samples={scene:samples};d.scene_lidar={scene:lidar};d.scene_annotations={scene:annotations}
    d.lidar_key={t:dataset.lidar_key[t] for t in tokens if t in dataset.lidar_key}
    table=dataset.tables
    d.tables={
        'sample':{s['token']:s for s in samples},
        'scene':{scene:table['scene'][scene]} if scene in table['scene'] else {},
        'ego_pose':{v['ego_pose_token']:table['ego_pose'][v['ego_pose_token']] for v in lidar},
        'calibrated_sensor':{v['calibrated_sensor_token']:table['calibrated_sensor'][v['calibrated_sensor_token']] for v in lidar},
        'instance':{v['instance_token']:table['instance'][v['instance_token']] for v in annotations},
        'category':table['category'],'log':table['log'],
    }
    return d

def work_scene(dataset,plans,tokens,out,cfg):
    from .evaluator import evaluate_sample
    if cfg.dataset=='nuscenes':
        # Cache maps per process across scenes, keyed by roots to avoid cross-dataset reuse.
        key=(str(dataset.root),cfg.nuscenes_map_root)
        dataset.maps=_WORKER_MAPS.setdefault(key,{})
    rows=[evaluate_sample(t,plans,dataset,out,cfg) for t in tokens]
    # Include reused maps too: loaders append provenance only on the first read.
    maps=dataset.input_report.get('map_files',[])
    if cfg.dataset=='nuscenes':
        from .inputs import digest
        maps=[{'path':str(m.path),'sha256':digest(m.path)} for m in dataset.maps.values()]
    return rows,maps

def run_scenes(dataset,plans,requested,out,cfg,workers):
    from .evaluator import evaluate_sample
    if workers==1:
        rows=[]
        for token in requested:
            row=evaluate_sample(token,plans,dataset,out,cfg);rows.append(row)
            report(row,len(rows),len(requested))
        return rows,1
    groups=OrderedDict()
    for t in requested:
        scene=dataset.scenario_for(t) if t in dataset.infos else '__unmatched__'
        groups.setdefault(scene,[]).append(t)
    count=min(workers,len(groups));jobs=iter(groups.items());done_rows={};map_files={}
    # Spawn avoids forking initialized IPOPT/BLAS state; cap native threads BEFORE imports.
    keys=('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')
    old={k:os.environ.get(k) for k in keys}
    for k in keys:os.environ[k]='1'
    try:
        with ProcessPoolExecutor(max_workers=count,mp_context=mp.get_context('spawn')) as pool:
            pending={}
            def submit():
                try:scene,tokens=next(jobs)
                except StopIteration:return False
                d=scene_dataset(dataset,scene,tokens)
                # Keep tensor runtimes out of worker serialization.
                from .prediction import array
                subset={}
                for t in tokens:
                    if t not in plans:continue
                    try:subset[t]=[array(v) for v in plans[t]] if isinstance(plans[t],(list,tuple)) and len(plans[t])==2 else plans[t]
                    except (ValueError,TypeError):subset[t]=plans[t]
                pending[pool.submit(work_scene,d,subset,tokens,out,cfg)]=scene
                return True
            for _ in range(count):submit()
            while pending:
                finished,_=wait(pending,return_when=FIRST_COMPLETED)
                for future in finished:
                    del pending[future]
                    rows,maps=future.result() # infrastructure failures must fail the run, not become model invalid
                    for row in rows:
                        done_rows[row['token']]=row;report(row,len(done_rows),len(requested))
                    for item in maps:map_files[item['path']]=item
                    submit()
    finally:
        for k,v in old.items():
            if v is None:os.environ.pop(k,None)
            else:os.environ[k]=v
    if cfg.dataset=='nuscenes':dataset.input_report['map_files']=[map_files[k] for k in sorted(map_files)]
    return [done_rows[t] for t in requested],count

def report(row,done,total):
    print(f'[{done}/{total}] {row["token"]}: '+(f'PDMS={row["PDMS"]:.4f}' if row['valid'] else row['invalid_reason']),flush=True)
