"""Timed Frenet MPC + soft single CLF; independent world bicycle integration.
Adapted from the supplied MPCControllerFrenetSingleCLF formulation.
Changes: speed is a state, acceleration a control, timed s/v reference,
zero-speed support, shrinking horizon, explicit solver failure.
"""
import os
# Avoid large BLAS thread pools in IPOPT on shared evaluators.
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import casadi as ca
import numpy as np
from scipy.interpolate import PchipInterpolator
from shapely.geometry import LineString, Point

class TrackingFailure(RuntimeError): pass

def wrap(a): return np.arctan2(np.sin(a),np.cos(a))

def reference(points,times,dt=.1):
    points=np.asarray(points,float); times=np.asarray(times,float)
    if points.shape != (len(times),2) or np.any(np.diff(times)<=0): raise ValueError('reference shape/timestamps')
    t=np.arange(31)*dt
    if times[0]>1e-6 or times[-1]<3-1e-6: raise ValueError('reference must cover 0..3 s')
    interp=PchipInterpolator(times,points,axis=0)
    xy=interp(t); vel=interp.derivative()(t); speed=np.linalg.norm(vel,axis=1)
    yaw=np.zeros(31)
    for k in range(31):
        yaw[k]=np.arctan2(vel[k,1],vel[k,0]) if speed[k]>.03 else (yaw[k-1] if k else 0.)
    return np.column_stack([xy,np.unwrap(yaw),speed])

class SpatialPath:
    def __init__(self,ref):
        xy=ref[:,:2]
        keep=np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-5]
        xy=xy[keep]
        if len(xy)<2: xy=np.array([ref[0,:2],ref[0,:2]+[1.,0.]])
        self.line=LineString(xy)
        self.s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        self.xy=xy
        self.yaw=np.unwrap(np.arctan2(np.gradient(xy[:,1],self.s),np.gradient(xy[:,0],self.s)))
        self.kappa=np.gradient(self.yaw,self.s)
        # Continuation is only geometry support; timed target ends at 3 seconds.
        end=xy[-1]+100*np.array([np.cos(self.yaw[-1]),np.sin(self.yaw[-1])])
        start=xy[0]-20*np.array([np.cos(self.yaw[0]),np.sin(self.yaw[0])])
        self.ext_s=np.r_[-20,self.s,self.s[-1]+100]
        self.ext_k=np.r_[self.kappa[0],self.kappa,self.kappa[-1]]
        self.ext_xy=np.vstack([start,xy,end])
    def project(self,state):
        s=float(self.line.project(Point(state[:2])))
        center=np.array(self.line.interpolate(s).coords[0]); yaw=np.interp(s,self.s,self.yaw)
        # Allow longitudinal departure beyond endpoints without clamping state.
        tangent=np.array([np.cos(yaw),np.sin(yaw)])
        extra=float((state[:2]-center)@tangent)
        if s<1e-6 or s>self.s[-1]-1e-6: s+=extra
        d=float((state[:2]-center)@np.array([-np.sin(yaw),np.cos(yaw)]))
        return np.array([s,d,wrap(state[2]-yaw),state[3]])

class FrenetCLF:
    def __init__(self,path,cfg):
        self.path=path; self.cfg=cfg; self.solvers={}
    def build(self,n):
        c=self.cfg.controller; dt=self.cfg.dt; veh=self.cfg.vehicle
        opt=ca.Opti(); x=opt.variable(4,n+1); u=opt.variable(2,n); slack=opt.variable(1,n)
        initial=opt.parameter(4); prevdelta=opt.parameter(); ref=opt.parameter(2,n+1)
        opt.subject_to(x[:,0]==initial)
        curvature=ca.interpolant('curvature','linear',[self.path.ext_s],self.path.ext_k)
        cost=0
        for j in range(n):
            s,d,e,v=x[:,j][0],x[:,j][1],x[:,j][2],x[:,j][3]
            delta,a=u[0,j],u[1,j]; kappa=curvature(s); denom=1-kappa*d
            opt.subject_to(denom>=.2)
            ds=v*ca.cos(e)/denom
            dx=ca.vertcat(ds,v*ca.sin(e),v/veh.wheelbase*ca.tan(delta)-kappa*ds,a)
            opt.subject_to(x[:,j+1]==x[:,j]+dt*dx)
            V=c.clf_d*d*d+c.clf_yaw*e*e
            Vnext=c.clf_d*x[1,j+1]**2+c.clf_yaw*x[2,j+1]**2
            opt.subject_to(Vnext-(1-c.clf_rate*dt)*V<=slack[0,j])
            opt.subject_to(slack[0,j]>=0)
            dprev=prevdelta if j==0 else u[0,j-1]
            rate=np.deg2rad(c.steer_rate_deg)*dt
            opt.subject_to(opt.bounded(-rate,delta-dprev,rate))
            cost+=c.r_steer*delta**2+c.r_accel*a**2+c.clf_slack_weight*slack[0,j]**2
        for j in range(n+1):
            cost+=c.q_s*(x[0,j]-ref[0,j])**2+c.q_d*x[1,j]**2+c.q_yaw*x[2,j]**2+c.q_v*(x[3,j]-ref[1,j])**2
        opt.subject_to(opt.bounded(0,x[3,:],c.v_max))
        opt.subject_to(opt.bounded(-np.deg2rad(veh.max_steer_deg),u[0,:],np.deg2rad(veh.max_steer_deg)))
        opt.subject_to(opt.bounded(c.a_min,u[1,:],c.a_max))
        opt.minimize(cost)
        opt.solver('ipopt',{'print_time':False},{'print_level':0,'sb':'yes','max_iter':c.max_iter,'tol':1e-6})
        self.solvers[n]=(opt,x,u,slack,initial,prevdelta,ref)
    def solve(self,state,previous_delta,target):
        n=target.shape[1]-1
        if n not in self.solvers: self.build(n)
        opt,x,u,slack,initial,prevdelta,ref=self.solvers[n]
        opt.set_value(initial,state); opt.set_value(prevdelta,previous_delta); opt.set_value(ref,target)
        opt.set_initial(x,np.vstack([target[0],np.linspace(state[1],0,n+1),np.linspace(state[2],0,n+1),target[1]]))
        opt.set_initial(u,0); opt.set_initial(slack,1.)
        try: sol=opt.solve()
        except RuntimeError as e: raise TrackingFailure(str(opt.stats().get('return_status','solver failure'))) from e
        return np.asarray(sol.value(u)).reshape(2,n)[:,0],float(np.max(sol.value(slack))),opt.stats()['return_status']

def rollout(ref,initial,cfg):
    path=SpatialPath(ref); tracker=FrenetCLF(path,cfg)
    target_s=np.array([path.line.project(Point(p)) for p in ref[:,:2]])
    states=np.zeros((31,4)); states[0]=initial
    controls=np.zeros((30,2)); slacks=[]; status=[]; delta_prev=0.
    for i in range(30):
        n=min(cfg.controller.horizon,30-i)
        target=np.vstack([target_s[i:i+n+1],ref[i:i+n+1,3]])
        command,slack,msg=tracker.solve(path.project(states[i]),delta_prev,target)
        delta,a=command; x,y,yaw,v=states[i]; dt=cfg.dt
        # Explicit Euler, same discretization as the supplied MPC dynamics.
        states[i+1]=[x+dt*v*np.cos(yaw),y+dt*v*np.sin(yaw),yaw+dt*v/cfg.vehicle.wheelbase*np.tan(delta),max(0,v+dt*a)]
        controls[i]=command; delta_prev=delta; slacks.append(slack);status.append(msg)
    return {'states':states,'controls':controls,'clf_slack':np.array(slacks),'solver_status':status,
            'tracking_rmse':float(np.sqrt(np.mean(np.sum((states[:,:2]-ref[:,:2])**2,axis=1))))}
