"""Export editable approximate map polygons. Review before validated evaluation."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
from shapely.geometry import LineString,mapping
from shapely.ops import unary_union

def export(hd_map,out,lane_width=3.5):
    df=pd.read_parquet(hd_map);features=[];lanes=[]
    for _,r in df[df['class']=='centerline'].iterrows():
        xy=np.asarray([np.asarray(p,float)[:2] for p in r['points']]);line=LineString(xy);poly=line.buffer(lane_width/2,cap_style=2);lanes.append(poly)
        features.append({'type':'Feature','properties':{'role':'lane','lane_id':str(r['id'])},'geometry':mapping(poly)})
    features.append({'type':'Feature','properties':{'role':'drivable'},'geometry':mapping(unary_union(lanes))})
    doc={'type':'FeatureCollection','properties':{'verified':False,'coordinate_frame':'HD map world metres','source':'UNREVIEWED centerline buffer; edit lane/drivable/intersection geometry before setting verified'},'features':features}
    Path(out).parent.mkdir(parents=True,exist_ok=True);Path(out).write_text(json.dumps(doc,ensure_ascii=False),encoding='utf-8')
