"""Inspect saved DAC geometry without rerunning evaluation."""
import argparse,json
from pathlib import Path
import numpy as np
from shapely.geometry import shape,Point
from etri_pdms.geometry import ego_box
from etri_pdms.config import Vehicle


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--token',required=True);a=p.parse_args()
    run=Path(a.run);folder=None
    for f in (run/'samples').glob('*/result.json'):
        r=json.loads(f.read_text())
        if r.get('token')==a.token:folder=f.parent;break
    if folder is None:p.error('token not found in run')
    scene=json.loads((folder/'scene.json').read_text());area=shape(scene['drivable']);veh=Vehicle(**scene['vehicle'])
    with np.load(folder/'trajectories.npz',allow_pickle=False) as z:
        for key in ('gt_raw','gt_rollout','pred_rollout'):
            first=None;bad=0;maximum=0.
            for i,state in enumerate(z[key]):
                corners=np.asarray(ego_box(state,veh).exterior.coords)[:4]
                outside=[(j,xy,Point(xy)) for j,xy in enumerate(corners) if not area.contains(Point(xy))]
                if not outside:continue
                bad+=1;maximum=max(maximum,max(point.distance(area) for _,_,point in outside))
                if first is None:
                    first={'frame':i,'time_s':round(i*.1,2),'corners':[{'index':j,'xy':xy.tolist(),
                        'distance_outside_m':point.distance(area),'on_boundary':area.covers(point)} for j,xy,point in outside]}
            print(json.dumps({'trajectory':key,'DAC_recomputed':int(bad==0),'failed_frames':bad,
                              'max_corner_outside_m':maximum,'first_failure':first},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
