from dataclasses import dataclass, asdict, field
import hashlib, json
import yaml

@dataclass
class Vehicle:
    length: float = 4.635
    width: float = 1.892
    height: float = 2.434
    wheelbase: float = 3.0
    front_overhang: float = 0.845
    rear_overhang: float = 0.790
    max_steer_deg: float = 40.0
    min_turning_radius_metadata: float = 5.87
    @property
    def front(self): return self.wheelbase + self.front_overhang
    @property
    def center_offset(self): return (self.front-self.rear_overhang)/2

@dataclass
class Controller:
    horizon: int = 15
    a_min: float = -4.0
    a_max: float = 2.0
    v_max: float = 40.0
    steer_rate_deg: float = 30.0
    q_s: float = 2.0
    q_d: float = 5.0
    q_yaw: float = 3.0
    q_v: float = 2.0
    r_steer: float = 0.1
    r_accel: float = 0.1
    clf_d: float = 0.5
    clf_yaw: float = 0.5
    clf_rate: float = 0.5
    clf_slack_weight: float = 1000.0
    max_iter: int = 150

@dataclass
class Config:
    dt: float = 0.1
    horizon_s: float = 3.0
    prediction_dt: float = 0.5
    representation: str = 'step_offsets'
    axes: str = 'x_forward_y_left'
    raw_timestamp_unit: str = 'ms'
    info_timestamp_unit: str = 'us'
    max_time_gap_s: float = 0.16
    map_mode: str = 'approximate'
    lane_width: float = 3.5
    route_endpoint_tolerance: float = 0.25
    object_length_column: str = 'width[m]'
    object_width_column: str = 'length[m]'
    vehicle: Vehicle = field(default_factory=Vehicle)
    controller: Controller = field(default_factory=Controller)
    def validate(self):
        if self.dt != .1 or self.horizon_s != 3 or self.prediction_dt != .5:
            raise ValueError('v0.1 contract: dt=.1, horizon=3, prediction_dt=.5')
        if self.map_mode not in ('approximate', 'validated'): raise ValueError('map_mode')
        if self.representation not in ('step_offsets','relative_positions'): raise ValueError('representation')
        if self.axes not in ('x_forward_y_left','x_right_y_forward'): raise ValueError('axes')
        v=self.vehicle
        if abs(v.length-v.front-v.rear_overhang)>1e-6: raise ValueError('vehicle length inconsistent')
        if not 0 < v.max_steer_deg < 80 or v.width<=0: raise ValueError('vehicle')
        if self.controller.horizon<1 or self.controller.a_min>=0 or self.controller.a_max<=0: raise ValueError('controller')
        return self
    def to_dict(self): return asdict(self)
    def hash(self): return hashlib.sha256(json.dumps(self.to_dict(),sort_keys=True).encode()).hexdigest()[:16]

def read_config(path=None):
    data=yaml.safe_load(open(path)) if path else {}
    data=data or {}
    data['vehicle']=Vehicle(**data.get('vehicle',{}))
    data['controller']=Controller(**data.get('controller',{}))
    return Config(**data).validate()
