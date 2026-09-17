"""Offline initial speed with a scene-start-only forward difference."""
import numpy as np

def initial_speed(t0,start,end,sample_xy,tolerance=1e-6):
    if not np.isfinite([t0,start,end]).all() or start>end:
        raise ValueError('Invalid initial-speed time bounds')
    if t0<start-tolerance or t0>end+tolerance:
        raise ValueError('initial_speed: current time outside scene pose coverage')
    forward=t0-.1<start-tolerance
    query=np.array([t0,t0+.1] if forward else [t0-.1,t0])
    # No catch-all fallback: gaps, corrupt poses and missing future stay invalid.
    points=np.asarray(sample_xy(query),float)
    if points.shape!=(2,2) or not np.isfinite(points).all():
        raise ValueError('Invalid initial-speed position samples')
    speed=float(np.linalg.norm(points[1]-points[0])/.1)
    return speed,{'initial_speed_mps':speed,
                  'initial_speed_source':'forward_pose_difference' if forward else 'backward_pose_difference',
                  'initial_speed_window_s':.1,'initial_speed_uses_future':bool(forward)}
