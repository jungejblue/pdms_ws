"""Current rear-axle cache contract, matching the supplied py123d parser."""
from collections.abc import Mapping
import numpy as np
from scipy.spatial.transform import Rotation

EXPECTED = {
    'coordinate_frame': 'current_ego_rear_axle',
    'coordinate_axes': 'x_forward_y_left_z_up',
    'ego_origin': 'rear_axle_center',
}

def pose_rotation(value):
    a=np.asarray(value,dtype=float)
    if a.shape==(3,3) or a.size==9:
        r=a.reshape(3,3)
        if not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(r),1,atol=1e-5):
            raise ValueError('Cache pose rotation must be a proper rotation matrix')
        return r
    q=a.reshape(-1)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-8:
        raise ValueError('Cache pose requires a 3x3 rotation or wxyz quaternion')
    return Rotation.from_quat(q[[1,2,3,0]]).as_matrix()

def cache_pose(info):
    meta=info.get('conversion_meta')
    if not isinstance(meta,Mapping):
        raise ValueError('Missing conversion_meta: cache mode requires py123d current_ego_rear_axle metadata; do not guess the frame')
    for key,expected in EXPECTED.items():
        if meta.get(key)!=expected:raise ValueError(f'Cache {key}: expected {expected}, got {meta.get(key)!r}')
    if meta.get('quaternion_order') not in (None,'wxyz'):
        raise ValueError('Cache quaternion_order must be wxyz')
    t=info.get('map_ego2global_translation')
    if t is None:t=info.get('ego2global_translation')
    r=info.get('map_ego2global_rotation')
    if r is None:r=info.get('ego2global_rotation')
    if t is None or r is None:raise ValueError('Cache requires ego2global pose (map_ego2global takes precedence)')
    t=np.asarray(t,float).reshape(-1)
    if t.shape!=(3,) or not np.isfinite(t).all():raise ValueError('Cache global translation must be finite XYZ')
    return pose_rotation(r),t
