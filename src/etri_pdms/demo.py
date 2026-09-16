"""Synthetic fixtures only; no synthetic result is a VAD benchmark score."""
from pathlib import Path
import pickle,json
import numpy as np
import pandas as pd
from shapely.geometry import mapping,box

def create_demo(root):
    root=Path(root);folder=root/'data'/'synthetic_straight';folder.mkdir(parents=True,exist_ok=True)
    t=np.arange(-1,6.01,.1); stamp=1_700_000_000_000+t*1000
    ego=pd.DataFrame({'timestamp':stamp,'x':t*5,'y':np.zeros(len(t)),'yaw':np.zeros(len(t))})
    ego.to_parquet(folder/'ego_pose.parquet',index=False);ego.to_parquet(folder/'hd_ego_pose.parquet',index=False)
    pd.DataFrame([{'id':1,'class':'centerline','points':[[-30.,0.],[10.,0.]]},{'id':2,'class':'centerline','points':[[10.,0.],[80.,0.]]}]).to_parquet(folder/'hd_map.parquet',index=False)
    objects=[]
    for ti,st in zip(t,stamp):
        objects.append({'timestamp':st,'class':'ego','obj_id':0,'x[m]':ti*5,'y[m]':0.,'heading[rad]':0.,'width[m]':4.635,'length[m]':1.892,'height[m]':2.434})
    pd.DataFrame(objects).to_parquet(folder/'object.parquet',index=False)
    features=[{'type':'Feature','properties':{'role':role},'geometry':mapping(box(-30,-2,80,2))} for role in ('lane','drivable')]
    (folder/'map_polygons.geojson').write_text(json.dumps({'type':'FeatureCollection','properties':{'verified':True,'source':'synthetic exact geometry'},'features':features}))
    plans={};infos=[]
    for i,kind in enumerate(['perfect','offroad','stopped_prediction']):
        token=f'synthetic_straight_{i:08d}';infos.append({'token':token,'scene_token':'synthetic_straight','timestamp':1_700_000_000_000_000})
        points=np.column_stack([np.arange(1,7)*2.5,np.zeros(6)])
        if kind=='offroad':points[:,1]=np.arange(1,7)*1.8
        if kind=='stopped_prediction':points*=0
        offsets=np.diff(np.vstack([[0,0],points]),axis=0)
        plans[token]=[np.stack([offsets]*3),np.array([[[[0,0,1]]]],float)]
    with open(root/'predictions.pkl','wb') as f:pickle.dump({'plan_results':plans,'meta':{'synthetic':True}},f)
    with open(root/'infos.pkl','wb') as f:pickle.dump({'infos':infos},f)
    return root/'predictions.pkl',root/'infos.pkl',root/'data'
