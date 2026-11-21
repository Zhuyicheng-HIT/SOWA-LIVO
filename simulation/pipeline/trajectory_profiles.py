"""Shared continuous kinematics for rendered sensors and independent analytic IMU.

The profile is fixed before estimator replay. No estimator consumes this module.
"""
import hashlib, math, pathlib
import numpy as np
from trajectory_kinematics import rotation, state
Q=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
V=1.8
RAMP=3.
TURN_T=8.
MAIN_DISTANCE=122.
RETURN_DISTANCE=77.
TURN_START=13.+RAMP+(MAIN_DISTANCE-V*RAMP/2)/V
TURN_END=TURN_START+TURN_T
DECEL_START=TURN_END+(RETURN_DISTANCE-V*RAMP/2)/V
STOP_TIME=DECEL_START+RAMP
DURATION=139.
CONTROL=np.array([[-5.,99.],[-5.,99.+V*TURN_T/5],[-5.,99.+2*V*TURN_T/5],
                  [1.-2*V*TURN_T/5,105.],[1.-V*TURN_T/5,105.],[1.,105.]])

def bezier(control,u):
    n=len(control)-1
    return sum(math.comb(n,k)*(1-u)**(n-k)*u**k*control[k] for k in range(n+1))

def hospital_l_state(t):
    t=float(t)
    if t<13.:
        p,r=state(t)
        return np.array([-5.,-23.,0.])+Q@(p+np.array([20.,0,0])),Q@r
    if t<TURN_START:
        f=t-13.;u=np.clip(f/RAMP,0.,1.)
        distance=V*RAMP*(u**3-.5*u**4)+V*max(f-RAMP,0.)
        return np.array([-5.,-23.+distance,1.55]),rotation(0.,0.,np.pi/2)
    if t<TURN_END:
        u=(t-TURN_START)/TURN_T;p=bezier(CONTROL,u)
        v=bezier(5*np.diff(CONTROL,axis=0),u)/TURN_T
        return np.array([*p,1.55]),rotation(0.,0.,np.arctan2(v[1],v[0]))
    if t<DECEL_START:
        distance=V*(t-TURN_END)
    else:
        u=np.clip((t-DECEL_START)/RAMP,0.,1.)
        distance=V*(DECEL_START-TURN_END)+V*RAMP*(u-u**3+.5*u**4)
    return np.array([1.+distance,105.,1.55]),rotation(0.,0.,0.)

def profile_metadata():
    return {'id':'hospital_l_v1','duration_s':DURATION,'speed_straight_mps':V,
        'static_s':[0.,5.],'initialization_motion_s':[5.,13.],
        'main_translation_s':[13.,TURN_START],'turn_s':[TURN_START,TURN_END],
        'return_translation_s':[TURN_END,STOP_TIME],'final_static_s':[STOP_TIME,DURATION],
        'turn_control_xy_m':CONTROL.tolist(),'world_origin_body_start':[-5.,-23.,1.55],
        'world_rotation_initial_body':Q.tolist(),
        'evaluation_windows_absolute_s':{'whole':[1016.,1138.],
            'main_middle':[1025.,1065.],'turn':[1000.+TURN_START,1000.+TURN_END],
            'return_middle':[1104.,1120.]},
        'evaluation_policy':'Same fixed 20 Hz grid, scale 1 and first-pose alignment; whole-run alignment retained for phase errors; additional phase-local RPE.',
        'source_module_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
        'initialization_source_sha256':hashlib.sha256(_source.read_bytes()).hexdigest()}
