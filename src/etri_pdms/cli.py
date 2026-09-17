"""Public command line interface for pdms_ws."""
import argparse
import json
import os
from pathlib import Path
from .config import read_config
from .prediction import load_plans,select_prediction
from .inputs import load_infos



def workspace_root():
    value = os.environ.get('PDMS_WS_ROOT')
    root = Path(value).expanduser() if value else Path(__file__).resolve().parents[2]
    return root.resolve()


def resolve_run_path(value):
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    root = workspace_root()
    runs = root / 'runs'
    target = (root / path if path.parts and path.parts[0] == 'runs' else runs / path).resolve()
    if not target.is_relative_to(runs.resolve()) or target == runs.resolve():
        raise ValueError('Use a run name under runs/, or an explicit absolute output path')
    return target


def prepare_args(args, parser):
    if args.action in ('inspect', 'evaluate'):
        nusc = args.dataset == 'nuscenes'
        args.pred = args.pred or (os.environ.get('NUSCENES_PREDICTION_CACHE') if nusc else (os.environ.get('ETRI_PREDICTION_CACHE') or os.environ.get('PDMS_PLANNING_PKL')))
        args.raw = args.raw or (os.environ.get('NUSCENES_CACHE_PATH') if nusc else (os.environ.get('ETRI_CACHE_PATH') or os.environ.get('PDMS_RAW_PKL')))
        args.data_root = args.data_root or os.environ.get('NUSCENES_DATA_ROOT' if nusc else 'PDMS_DATA_ROOT')
        required = [('pred', '--planning-pkl', 'NUSCENES_PREDICTION_CACHE' if nusc else 'ETRI_PREDICTION_CACHE', False)]
        if nusc or args.action == 'evaluate' or args.raw:
            required.append(('raw', '--raw-pkl', 'NUSCENES_CACHE_PATH' if nusc else 'ETRI_CACHE_PATH', False))
        if args.action == 'evaluate' or nusc:
            required.append(('data_root', '--data-root', 'PDMS_DATA_ROOT', True))
        for attr, option, env, directory in required:
            value = getattr(args, attr)
            if not value:
                parser.error(f'{option} or {env} is required; source scripts/setup_pdms_env.sh (Docker: setup_pdms_docker_env.sh)')
            path = Path(value).expanduser()
            if not (path.is_dir() if directory else (path.is_file() or path.is_dir())):
                parser.error(f'{option}: path not found: {path}')
            setattr(args, attr, str(path.resolve()))
        args.config = str(Path(args.config).expanduser())
        if not Path(args.config).is_file():
            parser.error(f'Config not found: {args.config}; set PDMS_WS_ROOT or pass --config')
    if args.action in ('evaluate', 'demo'):
        args.out = str(resolve_run_path(args.out))
    elif args.action == 'serve':
        args.run = str(resolve_run_path(args.run))


def main():
    parser=argparse.ArgumentParser(prog='pdms',description='ETRI/nuScenes adapted PDMS evaluation. Load only trusted pickle files.')
    commands=parser.add_subparsers(dest='action',required=True)
    check=commands.add_parser('inspect',help='Inspect planning PKL and raw/infos token compatibility')
    check.add_argument('--planning-pkl','--pred','--planning-dir',dest='pred',default=None)
    check.add_argument('--raw-pkl','--infos','--infos-dir',dest='raw',default=None)
    check.add_argument('--config',default=str(workspace_root()/'configs'/'ioniq5_2023.yaml'))
    evaluate=commands.add_parser('evaluate',help='Write scenario JSON, sample metrics and saved viewer data')
    evaluate.add_argument('--raw-pkl','--infos','--infos-dir',dest='raw',default=None)
    evaluate.add_argument('--planning-pkl','--pred','--planning-dir',dest='pred',default=None)
    evaluate.add_argument('--data-root',default=None,help='Original ETRI parquet or nuScenes metadata root')
    evaluate.add_argument('--out',required=True);evaluate.add_argument('--config',default=str(workspace_root()/'configs'/'ioniq5_2023.yaml'))
    evaluate.add_argument('--limit',type=int)
    evaluate.add_argument('--sample-interval',type=float,default=0.0,help='Seconds between scene samples; default 0 evaluates all time-eligible samples')
    evaluate.add_argument('--ep-reference',choices=['gt_path','centerline'],default=None)
    evaluate.add_argument('--workers',type=int,choices=range(1,5),default=1,help='CPU processes, 1..4; scene-parallel')
    evaluate.add_argument('--tokens',help='One exact sample token per line; use the same list for all models')
    evaluate.add_argument('--visualize',type=int,default=0,help='Optional static HTML/PNG exports; localhost does not require them')
    demo=commands.add_parser('demo',help='Generate synthetic raw/planning PKLs and evaluate them')
    demo.add_argument('--out',required=True)
    demo.add_argument('--visualize',type=int,default=0)
    viewer=commands.add_parser('serve',help='Browse saved evaluation data at localhost using Viser')
    viewer.add_argument('--run',required=True);viewer.add_argument('--host',default=os.environ.get('PDMS_HOST','127.0.0.1'));viewer.add_argument('--port',type=int,default=7200)
    visual=commands.add_parser('visualize',help='Export one sample to static HTML/PNG')
    visual.add_argument('--sample-dir',required=True)
    map_cmd=commands.add_parser('map-template',help='Export an unverified map polygon template for review')
    map_cmd.add_argument('--hd-map',required=True);map_cmd.add_argument('--out',required=True);map_cmd.add_argument('--lane-width',type=float,default=3.5)
    check.add_argument('--data-root', default=None)
    for command in (check, evaluate):
        command.add_argument('--dataset', choices=['etri','nuscenes'], default=os.environ.get('PDMS_DATASET','etri'))
        command.add_argument('--nuscenes-version', default=os.environ.get('NUSCENES_VERSION','v1.0-mini'))
        command.add_argument('--prediction-frame', choices=['cache','lidar','ego'], default=os.environ.get('NUSCENES_PREDICTION_FRAME','cache'))
        command.add_argument('--map-root', default=os.environ.get('NUSCENES_MAP_ROOT',''))
    args=parser.parse_args()
    try:return dispatch(args,parser)
    except (ValueError,FileNotFoundError,FileExistsError,ImportError,OSError) as exc:
        parser.exit(2,f'Error: {exc}\n')


def evaluation_config(args):
    cfg=read_config(args.config)
    if getattr(args,'ep_reference',None): cfg.ep_reference=args.ep_reference
    cfg.dataset=args.dataset
    cfg.nuscenes_version=args.nuscenes_version
    cfg.nuscenes_prediction_frame=args.prediction_frame
    cfg.nuscenes_map_root=args.map_root
    if cfg.dataset=='nuscenes':
        cfg.axes='x_forward_y_left' # frame conversion is performed by the dataset adapter
        if cfg.nuscenes_prediction_frame=='cache' and cfg.representation!='step_offsets':
            raise ValueError('py123d cache mode requires representation: step_offsets (cumsum exactly once)')
    cfg.validate()
    return cfg


def dispatch(args,parser):
    prepare_args(args,parser)
    if args.action=='serve':
        from .viewer import serve
        return serve(args.run,args.host,args.port)
    if args.action=='visualize':
        from .visualization import render_sample,render_png
        render_sample(args.sample_dir);render_png(args.sample_dir);return 0
    if args.action=='map-template':
        from .map_template import export
        export(args.hd_map,args.out,args.lane_width);return 0
    if args.action=='inspect':
        cfg=evaluation_config(args);plans,planning_report=load_plans(args.pred,return_report=True);errors=[];modes=[0,0,0]
        for token,entry in plans.items():
            try:_,mode=select_prediction(entry,cfg);modes[mode]+=1
            except (ValueError,TypeError) as exc:errors.append({'token':token,'error':str(exc)})
        result={'tokens':len(plans),'valid_entries':sum(modes),'command_counts':modes,'errors':errors[:20],
                'representation':cfg.representation,'axes':cfg.axes,'first_tokens':list(plans)[:3],
                'planning_inputs':{k:v for k,v in planning_report.items() if k != 'token_sources'}}
        if args.dataset=='nuscenes':
            from .nuscenes_data import NuScenesDataset
            dataset=NuScenesDataset(args.data_root,args.raw,cfg)
            tokens=set(dataset.infos)&set(dataset.tables['sample'])
            for token in sorted(set(plans)&tokens):
                try:dataset.check_coordinates(token)
                except (ValueError,KeyError,TypeError) as exc:errors.append({'token':token,'error':str(exc)})
            result['errors']=errors[:20]
            result['coordinate_error_count']=len(errors)
            result.update(dataset='nuscenes', prediction_frame=cfg.nuscenes_prediction_frame,
                          matching_dataset_tokens=len(set(plans)&tokens),
                          unmatched_prediction_tokens=len(set(plans)-tokens),
                          raw_tokens_without_predictions=len(tokens-set(plans)),
                          raw_inputs=dataset.input_report)
        if args.raw and args.dataset=='etri':
            entries, infos_report = load_infos(args.raw)
            result['infos_inputs'] = {k:v for k,v in infos_report.items() if k != 'token_sources'}
            tokens={str(i['token']) for i in entries};matched=set(plans)&tokens
            result.update(raw_format='infos',
                          matching_ETRI_tokens=len(matched),unmatched_prediction_tokens=len(set(plans)-tokens),
                          raw_tokens_without_predictions=len(tokens-set(plans)))
        print(json.dumps(result,indent=2,ensure_ascii=False))
        return 2 if errors or result.get('unmatched_prediction_tokens',0) else 0
    from .evaluator import evaluate
    if args.visualize<0:parser.error('--visualize must be nonnegative')
    if args.action=='demo':
        from .demo import create_demo
        root=Path(args.out).expanduser()
        if root.exists() and any(root.iterdir()):raise FileExistsError(f'Demo directory is not empty: {root}')
        pred,infos,data=create_demo(root);cfg=read_config();cfg.map_mode='validated'
        summary=evaluate(pred,infos,data,root/'evaluation',cfg,visualize=args.visualize)
    else:
        if args.limit is not None and args.limit<1:parser.error('--limit must be positive')
        tokens=[s.strip() for s in Path(args.tokens).read_text().splitlines() if s.strip()] if args.tokens else None
        summary=evaluate(args.pred,args.raw,args.data_root,args.out,evaluation_config(args),args.limit,tokens,args.visualize,args.workers,args.sample_interval)
    print(json.dumps(summary,indent=2,ensure_ascii=False))
    return 0 if summary['complete'] else 2

if __name__=='__main__':raise SystemExit(main())
