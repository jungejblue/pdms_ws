from pathlib import Path
import html,json
import numpy as np
from shapely.geometry import mapping,shape
from .geometry import ego_box

def scene_payload(sample,cfg):
    return {'ep_reference':cfg.ep_reference,'ep_path_info':sample.get('ep_path_info',{}),'dataset':cfg.dataset,'anchor_assumption':sample.get('anchor_assumption','configured_ego_rear_axle'),'route':mapping(sample['route']),'drivable':mapping(sample['drivable']),
            'objects':[[{'id':o['id'],'polygon':mapping(o['polygon'])} for o in frame] for frame in sample['objects'][:31]],
            'vehicle':cfg.to_dict()['vehicle'],'map_quality':sample['map_quality']}

def render_sample(folder):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    from .config import Vehicle
    folder=Path(folder);z=np.load(folder/'trajectories.npz');meta=json.load(open(folder/'result.json'));scene=json.load(open(folder/'scene.json'))
    vehicle=Vehicle(**scene['vehicle']);fig=make_subplots(rows=2,cols=2,specs=[[{'rowspan':2},{}],[None,{}]],subplot_titles=('BEV · metres','Speed · m/s','Steering · deg'),column_widths=[.65,.35])
    area=shape(scene['drivable']);geoms=list(area.geoms) if hasattr(area,'geoms') else [area]
    for i,g in enumerate(geoms):
        x,y=g.exterior.xy
        fig.add_trace(go.Scatter(x=list(x),y=list(y),fill='toself',fillcolor='rgba(120,150,140,.12)',line={'color':'#ccd5d0'},name='Drivable',showlegend=i==0),row=1,col=1)
    route=shape(scene['route']);x,y=route.xy
    fig.add_trace(go.Scatter(x=list(x),y=list(y),line={'color':'#789080','dash':'dot'},name='EP route'),row=1,col=1)
    for key,name,color,dash in [('pred_raw','VAD waypoints','#dd7c36','dot'),('pred_reference','VAD reference','#dd7c36','dash'),('gt_reference','GT reference','#3b9b81','dash'),('pred_rollout','VAD MPC','#c35420','solid'),('gt_rollout','GT MPC','#12685b','solid')]:
        pts=z[key]; fig.add_trace(go.Scatter(x=pts[:,0],y=pts[:,1],name=name,mode='lines+markers' if key=='pred_raw' else 'lines',line={'color':color,'dash':dash}),row=1,col=1)
    for key,label,color in [('pred','VAD','#c35420'),('gt','GT','#12685b')]:
        fig.add_trace(go.Scatter(x=np.arange(31)*.1,y=z[key+'_rollout'][:,3],name=label+' speed',line={'color':color}),row=1,col=2)
        fig.add_trace(go.Scatter(x=np.arange(30)*.1,y=np.rad2deg(z[key+'_controls'][:,0]),name=label+' steering',line={'color':color}),row=2,col=2)
    animated=[]
    for label in ['VAD vehicle','GT vehicle','Objects']:
        animated.append(len(fig.data));fig.add_trace(go.Scatter(x=[],y=[],mode='lines',name=label),row=1,col=1)
    frames=[]
    for i in range(31):
        traces=[]
        for key,color in [('pred_rollout','#c35420'),('gt_rollout','#12685b')]:
            poly=ego_box(z[key][i],vehicle);x,y=poly.exterior.xy
            traces.append(go.Scatter(x=list(x),y=list(y),line={'color':color,'width':3}))
        ox=[];oy=[]
        for obj in scene['objects'][i]:
            x,y=shape(obj['polygon']).exterior.xy;ox+=list(x)+[None];oy+=list(y)+[None]
        traces.append(go.Scatter(x=ox,y=oy,line={'color':'#53657c','width':2}))
        frames.append(go.Frame(name=str(i),data=traces,traces=animated))
    fig.frames=frames
    for idx,trace in zip(animated,frames[0].data):fig.data[idx].update(trace)
    scores=' · '.join(f'{k}={meta[k]:.3f}' for k in ('PDMS','NC','DAC','EP','TTC','C'))
    fig.update_layout(template='plotly_white',height=960,
        margin={'l':80,'r':60,'t':110,'b':240},title=f'{html.escape(meta["token"])}<br>{scores}<br>{scene["map_quality"]}',
        sliders=[{'y':0,'yanchor':'top','pad':{'t':40,'b':0},'steps':[{'method':'animate','args':[[str(i)],{'mode':'immediate','frame':{'duration':0,'redraw':True}}],'label':f'{i*.1:.1f}s'} for i in range(31)]}],
        legend={'orientation':'h','x':0,'xanchor':'left','y':-.26,'yanchor':'top'})
    points=np.vstack([z['pred_rollout'][:,:2],z['gt_rollout'][:,:2]])
    lo=points.min(axis=0)-12;hi=points.max(axis=0)+12
    fig.update_xaxes(range=[lo[0],hi[0]],row=1,col=1);fig.update_yaxes(range=[lo[1],hi[1]],scaleanchor='x',scaleratio=1,row=1,col=1)
    fig.write_html(folder/'visualization.html',include_plotlyjs=True)

def render_png(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(folder);z=np.load(folder/'trajectories.npz');scene=json.load(open(folder/'scene.json'))
    fig,ax=plt.subplots(figsize=(8,6));area=shape(scene['drivable'])
    for g in (area.geoms if hasattr(area,'geoms') else [area]):
        x,y=g.exterior.xy;ax.fill(x,y,color='#e2e9e5')
    for key,color,label,ls in [('gt_reference','#2e9477','GT reference','--'),('pred_reference','#ef9e65','VAD reference','--'),('gt_rollout','#12685b','GT MPC','-'),('pred_rollout','#c35420','VAD MPC','-')]:
        a=z[key];ax.plot(a[:,0],a[:,1],ls,color=color,label=label)
    a=z['pred_raw'];ax.scatter(a[:,0],a[:,1],color='#c35420')
    for obj in scene['objects'][0]:
        x,y=shape(obj['polygon']).exterior.xy;ax.plot(x,y,color='#66758a')
    points=np.vstack([z['pred_rollout'][:,:2],z['gt_rollout'][:,:2]])
    lo=points.min(axis=0)-8;hi=points.max(axis=0)+8
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='Forward x [m]',ylabel='Left y [m]',title=scene['map_quality']);ax.set_aspect('equal');ax.legend();fig.tight_layout();fig.savefig(folder/'bev.png',dpi=140);plt.close(fig)

def index_report(out,rows,summary):
    out=Path(out)
    cells=[]
    for r in rows:
        link=f'<a href="samples/{r["artifact_id"]}/visualization.html">View</a>' if r.get('valid') and r.get('visualized') else ''
        cells.append('<tr>'+''.join(f'<td>{html.escape(str(r.get(k,"")))}</td>' for k in ('token','valid','PDMS','NC','DAC','EP','TTC','C','invalid_reason'))+f'<td>{link}</td></tr>')
    text='<!doctype html><meta charset="utf-8"><title>PDMS</title><style>body{font:15px system-ui;margin:32px;background:#f6f8f7}table{border-collapse:collapse;background:white}td,th{padding:9px;border:1px solid #ddd}pre{white-space:pre-wrap}</style><h1>PDMS-GT-MPC</h1>'
    text+='<p>GT baseline · configured virtual vehicle · 3 s / 10 Hz. Dataset-adapted score, not official NAVSIM benchmark.</p><pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre><table><tr>'+''.join('<th>'+k+'</th>' for k in ('Token','Valid','PDMS','NC','DAC','EP','TTC','C','Reason','Visual'))+'</tr>'+''.join(cells)+'</table>'
    (out/'report.html').write_text(text,encoding='utf-8')
