#!/usr/bin/env python3
"""Capture a paused-step Gazebo hospital sensor rig along a prescribed 6DOF path.
Saves rendered RGB and instantaneous GPU LiDAR raw samples. No truth sent to estimator.
Use python3 -s to avoid unrelated user-site binary packages.
"""
import argparse,hashlib,json,os,pathlib,signal,subprocess,sys,threading,time,xml.etree.ElementTree as ET
import cv2,numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from google.protobuf import text_format
from trajectory_kinematics import quat, rotation, state

ROOT=pathlib.Path(__file__).resolve().parents[2]
SCENE=ROOT/'simulation/scenes/hospital_l_corridor_v1'
SCENE_ROOT=SCENE.parent
BASE=ROOT/'simulation/templates/hospital_world_template.sdf'
Q=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
def hospital_state(t):
 p,r=state(t);return np.array([-5.,-23.,0.])+Q@(p+np.array([20.,0,0])),Q@r

def ts(header):return header.stamp.sec+header.stamp.nsec*1e-9

def main(a):
 out=pathlib.Path(a.output)
 if out.exists() and any(out.iterdir()):raise SystemExit('Refuse nonempty capture output')
 out.mkdir(parents=True,exist_ok=True)
 capture_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
 base=pathlib.Path(a.world).resolve() if a.world else SCENE/('low.sdf' if a.texture=='low' else 'rich.sdf')
 state_fn=hospital_state;trajectory_metadata=None
 if a.trajectory=='hospital_l_v1':
  from trajectory_profiles import hospital_l_state,profile_metadata
  state_fn=hospital_l_state;trajectory_metadata=profile_metadata()
 elif a.trajectory=='hospital_l_excited_v2':
  from trajectory_profiles_v2 import hospital_l_excited_state,profile_metadata
  state_fn=hospital_l_excited_state;trajectory_metadata=profile_metadata()
 partition='doa_hospital_moving_'+str(os.getpid());os.environ['GZ_PARTITION']=partition
 env=dict(os.environ)
 resource_paths=[str(SCENE_ROOT)]+[p for p in env.get('GZ_SIM_RESOURCE_PATH','').split(os.pathsep) if p]
 env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join(resource_paths)
 env.update(LIBGL_ALWAYS_SOFTWARE='1',MESA_LOADER_DRIVER_OVERRIDE='swrast',GALLIUM_DRIVER='llvmpipe',EGL_PLATFORM='surfaceless')
 tree=ET.parse(base);world=tree.getroot().find('world');name=world.attrib['name']
 # Existing scene geometry/materials remain; only a sensor-only rig is added.
 rig=ET.fromstring('''<model name="doa_sensor_rig"><static>true</static><pose>-5 -23 1.55 0 0 1.57079632679</pose><link name="sensor_link">
 <sensor name="mono" type="camera"><pose>0.05 0 0.03 0 0 0</pose><always_on>true</always_on><update_rate>20</update_rate><topic>/doa/gz/image</topic><camera><horizontal_fov>1.3494818844</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.08</near><far>60</far></clip></camera></sensor>
 <sensor name="lidar" type="gpu_lidar"><always_on>true</always_on><update_rate>10</update_rate><topic>/doa/gz/lidar</topic><lidar><scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159265359</min_angle><max_angle>3.14159265359</max_angle></horizontal><vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.6</min_angle><max_angle>0.6</max_angle></vertical></scan><range><min>0.2</min><max>25</max><resolution>0.002</resolution></range></lidar></sensor>
 </link></model>''');world.append(rig)
 sdf=out/'capture_world.sdf';tree.write(sdf,encoding='unicode',xml_declaration=True)
 node=Node();lock=threading.Lock();images=[];scans=[];clock=[0.];callbacks=[];camera_info=[];actual_rig_poses=[]
 def imcb(m):
  with lock:images.append((ts(m.header),m.width,m.height,m.step,bytes(m.data)))
 def licb(m):
  with lock:scans.append((ts(m.header),np.asarray(m.ranges,dtype=np.float32),m.count,m.vertical_count,m.angle_min,m.angle_max,m.vertical_angle_min,m.vertical_angle_max,[m.world_pose.position.x,m.world_pose.position.y,m.world_pose.position.z,m.world_pose.orientation.x,m.world_pose.orientation.y,m.world_pose.orientation.z,m.world_pose.orientation.w]))
 def infocb(m):
  camera_info[:]=[{'width':m.width,'height':m.height,'K':list(m.intrinsics.k),'distortion':list(m.distortion.k)}]
 def posecb(m):
  for p in m.pose:
   if p.name=='doa_sensor_rig':actual_rig_poses.append([ts(m.header),p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
 def ccb(m):clock[0]=m.sim.sec+m.sim.nsec*1e-9
 node.subscribe(Pose_V,'/world/'+name+'/pose/info',posecb);node.subscribe(CameraInfo,'/doa/gz/camera_info',infocb);node.subscribe(Image,'/doa/gz/image',imcb);node.subscribe(LaserScan,'/doa/gz/lidar',licb);node.subscribe(Clock,'/world/'+name+'/clock',ccb)
 log=open(out/'gazebo.log','w');proc=subprocess.Popen(['gz','sim','-s','--headless-rendering',str(sdf)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 broker=subprocess.Popen([sys.executable,'-s',str(pathlib.Path(__file__).parent/'gazebo_service_broker.py')],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
 def request(service,req,typ):
  broker.stdin.write(json.dumps({'service':service,'type':req.DESCRIPTOR.full_name,'text':text_format.MessageToString(req)})+'\n');broker.stdin.flush()
  reply=json.loads(broker.stdout.readline())
  if not reply['ok']:print('SERVICE_FAIL',service,reply,flush=True)
  return reply['ok']
 def step_to(target):
  req=WorldControl();req.pause=False;req.run_to_sim_time.sec=int(target);req.run_to_sim_time.nsec=int(round((target-int(target))*1e9))
  if not request('/world/'+name+'/control',req,WorldControl):
   # Idempotent absolute run-until request: safe retry if reply observation failed.
   if clock[0]>=target-1e-6:return
   if not request('/world/'+name+'/control',req,WorldControl):raise RuntimeError('Gazebo run-until rejected')
 def pose_at(t):
  p,R=state_fn(t);q=quat(R);req=Pose();req.name='doa_sensor_rig'
  req.position.x,req.position.y,req.position.z=p
  req.orientation.x,req.orientation.y,req.orientation.z,req.orientation.w=q
  for attempt in range(3):
   if request('/world/'+name+'/set_pose',req,Pose):return
   time.sleep(.1)
  raise RuntimeError('Gazebo pose rejected')
 started=time.monotonic();saved_images=[];saved_rgb=[];saved_scans=[];features=[];last_i=-1.;last_l=-1.;error=None
 try:
  # Wait for UserCommands service, without advancing an unobserved trajectory.
  for _ in range(40):
   try:
    if request('/world/'+name+'/control',WorldControl(pause=True),WorldControl):break
   except Exception:pass
   if proc.poll() is not None:raise RuntimeError('Gazebo exited during startup')
   time.sleep(.25)
  else:raise RuntimeError('Gazebo control service unavailable')
  pose_at(.001);step_to(.001)
  wait=time.monotonic()+15
  while not images or not scans:
   if time.monotonic()>wait:raise RuntimeError('Initial sensor messages timeout')
   time.sleep(.01)
  for index in range(int(a.duration*20)+1):
   target=.001+index*.05
   if index:
    pose_at(target);step_to(target)
   wait=time.monotonic()+15
   while clock[0]<target-1e-6:
    if time.monotonic()>wait:raise RuntimeError('Simulation clock failed to reach target')
    time.sleep(.002)
   # Rendering runs asynchronously after paused stepping. Require image at target.
   while not any(abs(v[0]-target)<.002 for v in images):
    if time.monotonic()>wait:raise RuntimeError('Image missing at requested paused tick '+str(target))
    time.sleep(.005)
   if index%2==0:
    while not any(abs(v[0]-target)<.002 for v in scans):
     if time.monotonic()>wait:raise RuntimeError('Flash cloud missing at requested paused tick '+str(target))
     time.sleep(.005)
   with lock:
    batch_i=images[:];images.clear();batch_l=scans[:];scans.clear()
   for t,w,h,s,data in batch_i:
    if t<=last_i:continue
    # At each acquisition tick, set_pose rig target equals this sample time.
    if abs(t-target)>.002:raise RuntimeError('Unmatched image pose timestamp')
    rgb=np.frombuffer(data,np.uint8).reshape(h,s)[:,:w*3].reshape(h,w,3);mono=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    rgb_file='rgb_%06d.png'%len(saved_images)
    if not cv2.imwrite(str(out/rgb_file),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):
     raise RuntimeError('RGB image write failed: '+rgb_file)
    saved_rgb.append([t,rgb_file])
    file='image_%06d.png'%len(saved_images);cv2.imwrite(str(out/file),mono);saved_images.append([t,file]);last_i=t
    if len(saved_images)%20==1:
     corners=cv2.goodFeaturesToTrack(mono,1000,.01,8);features.append([t,0 if corners is None else len(corners)])
   for t,ranges,nv,nh,amin,amax,vmin,vmax,actual_pose in batch_l:
    if t<=last_l:continue
    if abs(t-target)>.002:raise RuntimeError('Unmatched LiDAR pose timestamp')
    file='scan_%06d.npz'%len(saved_scans);np.savez_compressed(out/file,ranges=ranges,count=nv,vertical_count=nh,angle_min=amin,angle_max=amax,vertical_angle_min=vmin,vertical_angle_max=vmax,actual_world_pose=actual_pose)
    saved_scans.append([t,file]);last_l=t
   if index%100==0:print('Captured',target,'images',len(saved_images),'flash scans',len(saved_scans),flush=True)
 except Exception as exc:error=repr(exc);raise
 finally:
  cleanup=[]
  # A stuck renderer must neither survive the task nor prevent metadata writing.
  for sig,timeout in [(signal.SIGINT,8),(signal.SIGTERM,5),(signal.SIGKILL,3)]:
   if proc.poll() is not None:break
   cleanup.append(signal.Signals(sig).name)
   try:os.killpg(proc.pid,sig)
   except ProcessLookupError:pass
   try:proc.wait(timeout=timeout)
   except subprocess.TimeoutExpired:continue
  if proc.poll() is None:cleanup.append('renderer still present after SIGKILL')
  if broker.poll() is None:
   broker.terminate()
   try:broker.wait(timeout=3)
   except subprocess.TimeoutExpired:
    broker.kill()
    try:broker.wait(timeout=3)
    except subprocess.TimeoutExpired:cleanup.append('broker still present after SIGKILL')
  log.close()
  try:source_world=base.relative_to(ROOT).as_posix()
  except ValueError:source_world=base.name
  m={'status':'capture_complete' if error is None else 'capture_failed','error':error,'source_world':source_world,
    'source_world_sha256':hashlib.sha256(base.read_bytes()).hexdigest(),'partition':partition,'duration_requested_s':a.duration,'texture':a.texture,
    'images':saved_images,'rgb_images':saved_rgb,'raw_rgb_encoding':'rgb8','gray_policy':'mono8 derived from same RGB acquisition; no RGB reconstruction from grayscale',
    'scans':saved_scans,'image_feature_counts':features,'wall_seconds':time.monotonic()-started,
    'actual_camera_info':camera_info,'actual_rig_pose_samples':actual_rig_poses,'sensor_model':'Gazebo actual rendered monocular image and instantaneous GPU LiDAR; no fake spinning times',
    'trajectory':'Shared deterministic kinematic trajectory; continuous IMU computed offline from same trajectory',
    'trajectory_profile':a.trajectory,'trajectory_metadata':trajectory_metadata,
    'world_origin_body_start':[-5,-23,1.55],'world_rotation_initial_body':Q.tolist(),'process_exit':proc.returncode,
    'capture_script_sha256':capture_sha256,'cleanup_signals':cleanup,'broker_exit':broker.returncode}
  (out/'raw_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print(json.dumps({k:v for k,v in m.items() if k not in ['images','rgb_images','scans','image_feature_counts','actual_rig_pose_samples']},indent=2))

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--duration',type=float,default=43.);ap.add_argument('--texture',choices=['low','rich'],default='rich');ap.add_argument('--world',help='Optional derived hospital SDF; original world remains unchanged');ap.add_argument('--trajectory',choices=['hospital_straight_v1','hospital_l_v1','hospital_l_excited_v2'],default='hospital_straight_v1');main(ap.parse_args())
