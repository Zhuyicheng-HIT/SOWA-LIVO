"""Same corridor path with bounded, persistent cross-axis inertial excitation."""
import hashlib,pathlib
import numpy as np
from trajectory_profiles import hospital_l_state,profile_metadata as base_metadata,STOP_TIME

def smooth(u):
    u=np.clip(float(u),0.,1.)
    return 10*u**3-15*u**4+6*u**5

def hospital_l_excited_state(t):
    p,r=hospital_l_state(t);p=p.copy()
    if t>=13.:
        envelope=smooth((t-13.)/3.)*smooth((STOP_TIME-t)/3.)
        lateral=.15*np.sin(2*np.pi*(t-13.)/5.)*envelope
        vertical=.08*np.sin(2*np.pi*(t-13.)/3.7)*envelope
        p+=r[:,1]*lateral+np.array([0.,0.,vertical])
    return p,r

def profile_metadata():
    result=base_metadata()
    result['base_profile_source_sha256']=result.pop('source_module_sha256')
    result.update(id='hospital_l_excited_v2',
        source_module_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
        excitation={'lateral_amplitude_m':.15,'lateral_period_s':5.,'vertical_amplitude_m':.08,
                    'vertical_period_s':3.7,'onset_s':13.,'envelope_ramp_s':3.,'offset_s':STOP_TIME,
                    'orientation':'base corridor centerline heading; same for every estimator'},
        rationale='Maintain nonconstant inertial excitation in the geometry-poor long straight; predeclared for all sensor/algorithm modes before replay',
        component_frame='reference corridor centerline forward/right/up, evaluated offline')
    return result
