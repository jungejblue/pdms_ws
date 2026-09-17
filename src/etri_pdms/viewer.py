"""Viser localhost viewer for saved PDMS runs (no scoring or simulation)."""
from pathlib import Path
import argparse
import json
import socket
import threading
import time

import numpy as np
from shapely.geometry import shape
from shapely.ops import triangulate

from .config import Vehicle
from .geometry import ego_box

COLORS = {'pred': (220, 107, 45), 'gt': (33, 157, 128),
          'route': (91, 130, 197), 'objects': (120, 140, 170),
          'drivable': (124, 155, 137)}


def discover_samples(run):
    run = Path(run).expanduser().resolve()
    if not (run / 'samples').is_dir():
        raise ValueError(f'Expected an evaluation directory containing samples/: {run}')
    records = []
    for path in sorted((run / 'samples').glob('*/result.json')):
        result = json.loads(path.read_text())
        result['_folder'] = path.parent
        records.append(result)
    if not records:
        raise ValueError(f'No samples/*/result.json found under {run}')
    return sorted(records, key=lambda r: (not r.get('valid', False), r.get('PDMS') if r.get('PDMS') is not None else 2, str(r.get('token', ''))))


def polyline_segments(xy, z=0.04, dashed=False):
    xy = np.asarray(xy, dtype=np.float32)
    if len(xy) < 2:
        return np.empty((0, 2, 3), dtype=np.float32)
    segments = []
    for a, b in zip(xy[:-1, :2], xy[1:, :2]):
        if dashed:
            n = max(1, int(np.ceil(np.linalg.norm(b-a) / 0.35)))
            points = np.linspace(a, b, n+1)
            segments.extend(np.stack([points[:-1], points[1:]], axis=1)[::2])
        else:
            segments.append([a, b])
    out = np.zeros((len(segments), 2, 3), np.float32)
    out[:, :, :2] = segments
    out[:, :, 2] = z
    return out


def polygon_segments(geometry, z=0.01):
    polys = list(geometry.geoms) if hasattr(geometry, 'geoms') else [geometry]
    parts = []
    for poly in polys:
        if poly.is_empty or poly.geom_type != 'Polygon':
            continue
        parts.append(polyline_segments(poly.exterior.coords, z))
        parts.extend(polyline_segments(r.coords, z) for r in poly.interiors)
    return np.concatenate(parts) if parts else np.empty((0, 2, 3), np.float32)


def polygon_mesh(geometry):
    """Keep polygon holes; reject Delaunay triangles outside the drivable union."""
    polys = list(geometry.geoms) if hasattr(geometry, 'geoms') else [geometry]
    triangles = []
    for poly in polys:
        if poly.is_empty or poly.geom_type != 'Polygon':
            continue
        triangles.extend(np.asarray(t.exterior.coords)[:3, :2] for t in triangulate(poly) if poly.covers(t))
    if not triangles:
        return None
    xy = np.asarray(triangles, np.float32).reshape(-1, 2)
    vertices = np.column_stack([xy, np.zeros(len(xy), np.float32)])
    return vertices, np.arange(len(vertices), dtype=np.uint32).reshape(-1, 3)


def load_sample(record):
    folder = record['_folder']
    required = ('scene.json', 'trajectories.npz')
    if not record.get('valid') or not all((folder/p).is_file() for p in required):
        return None
    scene = json.loads((folder/'scene.json').read_text())
    with np.load(folder/'trajectories.npz', allow_pickle=False) as archive:
        arrays = {k: archive[k].copy() for k in archive.files}
    states = arrays['pred_rollout']
    if len(states) != len(arrays['gt_rollout']) or len(states) != len(scene['objects']):
        raise ValueError('Saved trajectory/object frame counts do not match')
    diagnostics = json.loads((folder/'diagnostics.json').read_text()) if (folder/'diagnostics.json').exists() else {}
    return {'scene': scene, 'arrays': arrays, 'vehicle': Vehicle(**scene['vehicle']), 'diagnostics': diagnostics}


class PDMSViewer:
    """Controls are shared by connected clients; camera poses are client-local."""
    def __init__(self, server, records):
        self.server = server
        self.records = records
        self.lock = threading.RLock()
        self.current = None
        self.handles = []
        self.static_handles = {}
        self.loading = False
        self.index = 0
        self.labels = [f'{i+1:04d} | {r["token"]} | '+(f'PDMS {r["PDMS"]:.3f}' if r.get('valid') else 'INVALID') for i,r in enumerate(records)]
        server.scene.set_up_direction('+z')
        server.scene.world_axes.visible = True
        server.gui.configure_theme(control_layout='collapsible', control_width='large', show_share_button=False)
        server.gui.add_markdown('## PDMS viewer\nGT baseline · configured virtual vehicle · saved rollout')
        with server.gui.add_folder('Sample'):
            self.selector = server.gui.add_dropdown('Sample', options=self.labels, initial_value=self.labels[0])
            self.previous = server.gui.add_button('Previous sample')
            self.next = server.gui.add_button('Next sample')
            self.search = server.gui.add_text('Token / ID', initial_value='')
            self.jump = server.gui.add_button('Jump to token')
            self.search_status = server.gui.add_markdown('')
            self.top_view = server.gui.add_button('Reset BEV camera')
        with server.gui.add_folder('Playback'):
            self.frame = server.gui.add_slider('Frame', min=0, max=30, step=1, initial_value=0)
            self.play = server.gui.add_checkbox('Play', initial_value=False)
            self.loop = server.gui.add_checkbox('Loop', initial_value=True)
            self.speed = server.gui.add_dropdown('Speed', options=('0.5x','1x','2x'), initial_value='1x')
            self.now = server.gui.add_markdown('Time: **0.0 s**')
        with server.gui.add_folder('Layers / legend'):
            server.gui.add_markdown('**Orange:** VAD · **Green:** GT · **Blue:** EP route\n\nSolid: rollout · Dashed: reference · Points: raw prediction')
            self.layers = {key: server.gui.add_checkbox(label, initial_value=True) for key,label in [
                ('drivable','Drivable area'),('route','EP route'),('pred','VAD rollout / ego'),
                ('gt','GT rollout / ego'),('reference','Reference paths'),('waypoints','VAD waypoints'),('objects','Objects')]}
        with server.gui.add_folder('Scores and diagnostics'):
            self.metrics = server.gui.add_markdown('Loading...')
            self.status = server.gui.add_markdown('')
        with server.gui.add_folder('Control charts', expand_by_default=False):
            import plotly.graph_objects as go
            self.charts = {name: server.gui.add_plotly(go.Figure(), aspect=1.5) for name in ('Speed [m/s]','Acceleration [m/s²]','Steering [deg]')}
        self.selector.on_update(self._select_event)
        self.frame.on_update(self._time_event)
        self.previous.on_click(lambda event: self.select((self.index-1) % len(self.records)))
        self.next.on_click(lambda event: self.select((self.index+1) % len(self.records)))
        self.jump.on_click(self._jump_event)
        self.top_view.on_click(lambda event: self.reset_cameras())
        for handle in self.layers.values():
            handle.on_update(lambda event: self.apply_visibility())
        server.on_client_connect(self.reset_camera)
        self.select(0)

    def _jump_event(self, event):
        query = self.search.value.strip()
        if not query:
            self.search_status.content = 'Enter a token or sample ID.'
            return
        matches = [i for i,r in enumerate(self.records) if query == str(r['token']) or query == r['_folder'].name]
        if not matches:
            matches = [i for i,r in enumerate(self.records) if query in str(r['token'])]
        if len(matches) == 1:
            self.search_status.content = ''
            self.select(matches[0])
        else:
            self.search_status.content = f'{len(matches)} matches. Use an exact token.'

    def _select_event(self, event):
        if not self.loading:
            self.select(self.labels.index(self.selector.value))

    def _time_event(self, event):
        if not self.loading:
            self.update_frame()

    def _line(self, name, segments, color, width=3.0):
        if not len(segments):
            return None
        handle = self.server.scene.add_line_segments('/pdms/'+name, points=segments, colors=color, thickness=width, thickness_units='screen')
        self.handles.append(handle)
        return handle

    def select(self, index):
        with self.lock, self.server.atomic():
            self.loading = True
            try:
                self.play.value = False
                self.index = index
                self.selector.value = self.labels[index]
                self.frame.value = 0
                for handle in self.handles:
                    handle.remove()
                self.handles.clear()
                self.static_handles.clear()
                self.current = None
                record = self.records[index]
                self.current = load_sample(record)
                self._update_scores(record)
                if self.current is None:
                    self.frame.disabled = True
                    self.play.disabled = True
                    self.status.content = str(record.get('invalid_reason') or 'Saved trajectory / scene data is unavailable.')
                    self.now.content = 'No playable trajectory.'
                    import plotly.graph_objects as go
                    for chart in self.charts.values():
                        chart.figure = go.Figure()
                    return
                self.frame.disabled = False
                self.play.disabled = False
                data = self.current
                arrays = data['arrays']
                self.frame.max = len(arrays['pred_rollout'])-1
                area = shape(data['scene']['drivable'])
                mesh = polygon_mesh(area)
                area_handles = []
                if mesh:
                    handle = self.server.scene.add_mesh_simple('/pdms/drivable_fill', vertices=mesh[0], faces=mesh[1], color=COLORS['drivable'], opacity=.18, side='double')
                    self.handles.append(handle); area_handles.append(handle)
                area_handles.append(self._line('drivable_edge',polygon_segments(area),COLORS['drivable'],1.0))
                self.static_handles['drivable'] = area_handles
                route = shape(data['scene']['route'])
                self.static_handles['route'] = [self._line('route',polyline_segments(route.coords,.025,True),COLORS['route'],2.0)]
                for key in ('pred','gt'):
                    self.static_handles[key] = [self._line(key+'_rollout',polyline_segments(arrays[key+'_rollout'][:,:2],.09),COLORS[key],3.0)]
                self.static_handles['reference'] = [self._line(key+'_reference',polyline_segments(arrays[key+'_reference'][:,:2],.06,True),COLORS[key],1.5) for key in ('pred','gt')]
                raw = arrays['pred_raw']; points = np.column_stack([raw[:,:2],np.full(len(raw),.13)])
                handle = self.server.scene.add_point_cloud('/pdms/waypoints',points=points.astype(np.float32),colors=COLORS['pred'],point_size=.16)
                self.handles.append(handle); self.static_handles['waypoints']=[handle]
                self.status.content = self._diagnostic_text(data)
                self._update_charts(data)
                self.update_frame()
                self.apply_visibility()
                self.reset_cameras()
            except (ValueError,KeyError,OSError,TypeError) as exc:
                for handle in self.handles:
                    handle.remove()
                self.handles.clear();self.static_handles.clear();self.current=None
                self.frame.disabled=True;self.play.disabled=True
                self.status.content=f'Viewer input error: {exc}'
            finally:
                self.loading = False

    def _update_scores(self, record):
        text = f'**{record["token"]}**\n\n'
        text += f'Map: `{record.get("map_quality", "unknown")}`\n\n'
        if record.get('dataset') == 'nuscenes':
            text += f"Dataset: nuScenes · {record.get('scene_name', '')}\n\nVirtual vehicle anchor: `{record.get('anchor_assumption', '')}`\n\n"
        if record.get('map_quality') == 'approximate_centerline_buffer':
            text += '**Approximate map — development score.**\n\n'
        if not record.get('valid'):
            self.metrics.content=text+'**INVALID — no score**'
            return
        text += '| Metric | VAD | GT |\n|---|---:|---:|\n'
        for key in ('PDMS','NC','DAC','EP','TTC','C'):
            gt=record.get('gt_'+key)
            gt_text=f'{gt:.3f}' if gt is not None else '—'
            text += f'| {key} | {record[key]:.3f} | {gt_text} |\n'
        if record.get('reference_failure'):
            text += '\n**GT baseline NC/DAC failure.**\n'
        for key in ('tracking_rmse','gt_tracking_rmse','raw_ADE','raw_FDE'):
            if record.get(key) is not None:
                text += f'\n{key}: {record[key]:.4f} m\n'
        self.metrics.content = text

    def _diagnostic_text(self, data):
        lines=[]
        for who in ('model','gt'):
            events=data['diagnostics'].get(who,{}).get('events',[])
            first={}
            for event in events:
                key=event['metric']
                if key not in first:first[key]=event
            for key,event in first.items():
                lines.append(f'{who} {key}: {event["time_s"]:.1f}s, track {event.get("track_id","—")}')
        return '\n\n'.join(lines) or 'No recorded NC/DAC/TTC events.'

    def _update_charts(self, data):
        import plotly.graph_objects as go
        for metric,handle in self.charts.items():
            fig=go.Figure()
            for key in ('pred','gt'):
                if metric.startswith('Speed'):
                    y=data['arrays'][key+'_rollout'][:,3]
                elif metric.startswith('Acceleration'):
                    y=data['arrays'][key+'_controls'][:,1]
                else:
                    y=np.rad2deg(data['arrays'][key+'_controls'][:,0])
                fig.add_trace(go.Scatter(x=(np.arange(len(y))*.1).tolist(),y=y.tolist(),name='VAD' if key=='pred' else 'GT',line={'color':'rgb'+str(COLORS[key])}))
            fig.update_layout(title=metric,template='plotly_white',margin={'l':45,'r':10,'t':35,'b':40},legend={'orientation':'h','y':-.25},xaxis_title='Time [s]')
            # Prevent tiny numerical differences from filling the whole speed axis.
            all_y=np.concatenate([np.asarray(trace.y) for trace in fig.data])
            lower=float(all_y.min());upper=float(all_y.max())
            pad=max(.1 if metric.startswith('Speed') else .2,(upper-lower)*.1)
            fig.update_yaxes(range=[lower-pad,upper+pad])
            handle.figure=fig

    def update_frame(self):
        with self.lock, self.server.atomic():
            if self.current is None:return
            data=self.current;index=int(self.frame.value)
            for key in ('pred','gt'):
                name=key+'_ego'
                for old in self.static_handles.get(name,[]):
                    if old is not None:
                        old.remove();self.handles = [h for h in self.handles if h is not old]
                state=data['arrays'][key+'_rollout'][index]
                handle=self._line(name,polygon_segments(ego_box(state,data['vehicle']),.16),COLORS[key],4.)
                self.static_handles[name]=[handle]
            for old in self.static_handles.get('objects',[]):
                if old is not None:old.remove();self.handles = [h for h in self.handles if h is not old]
            parts=[polygon_segments(shape(obj['polygon']),.12) for obj in data['scene']['objects'][index]]
            segments=np.concatenate(parts) if parts else np.empty((0,2,3),np.float32)
            self.static_handles['objects']=[self._line('objects',segments,COLORS['objects'],2.)]
            ps=data['arrays']['pred_rollout'][index,3];gs=data['arrays']['gt_rollout'][index,3]
            self.now.content=f'Time: **{index*.1:.1f} s** · frame {index}/{self.frame.max}\n\nVAD: {ps:.2f} m/s · GT: {gs:.2f} m/s'
            self.apply_visibility()

    def apply_visibility(self):
        with self.lock:
            for key,handles in self.static_handles.items():
                layer=key.replace('_ego','')
                visible=self.layers[layer].value
                for handle in handles:
                    if handle is not None:handle.visible=visible

    def reset_camera(self, client):
        with self.lock:
            if self.current is None:return
            arrays=self.current['arrays']
            points=np.vstack([arrays[k+'_rollout'][:,:2] for k in ('pred','gt')])
            low=points.min(axis=0)-7;high=points.max(axis=0)+7
            mid=(low+high)/2;span=max(high-low)
            client.camera.up_direction=(0.,1.,0.)
            client.camera.position=(float(mid[0]),float(mid[1]),float(max(25,span*1.6)))
            client.camera.look_at=(float(mid[0]),float(mid[1]),0.)

    def reset_cameras(self):
        for client in self.server.get_clients().values():self.reset_camera(client)

    def tick(self, elapsed):
        with self.lock:
            if self.current is None or not self.play.value:return
            rate=float(self.speed.value[:-1]);interval=.1/rate
            if elapsed<interval:return
            frame=int(self.frame.value)+1
            if frame>self.frame.max:
                if self.loop.value:frame=0
                else:self.play.value=False;return
            # Viser invokes on_update for programmatic changes as well.
            self.frame.value=frame
            return True


def serve(run, host='127.0.0.1', port=7200):
    try:
        import viser
    except ImportError as exc:
        raise RuntimeError('Install viewer dependencies: python -m pip install -e ".[viewer]"') from exc
    records=discover_samples(run)
    if not 1<=port<=65535:raise ValueError('port must be in 1..65535')
    # Viser may otherwise select another port automatically; make CLI port explicit.
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as check:
        check.bind((host,port))
    server=viser.ViserServer(host=host,port=port,label='PDMS · GT / prediction')
    try:
        viewer=PDMSViewer(server,records)
        display_host='localhost' if host in ('127.0.0.1','0.0.0.0') else host
        print(f'PDMS viewer: http://{display_host}:{server.get_port()} | {len(records)} samples | Ctrl+C to stop',flush=True)
        last=time.monotonic()
        while True:
            now=time.monotonic()
            if viewer.tick(now-last):last=now
            if not viewer.play.value:last=now
            time.sleep(.01)
    except KeyboardInterrupt:
        print('\nStopping PDMS viewer.',flush=True)
    finally:
        server.stop()
    return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=7200)
    args=parser.parse_args()
    return serve(args.run,args.host,args.port)

if __name__=='__main__':raise SystemExit(main())
