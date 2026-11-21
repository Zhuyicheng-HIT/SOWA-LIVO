#!/usr/bin/env python3
"""Convert frozen Gazebo raw actual frames/flashes to ROS1 sensor-only bag.
This simulator uses kinematic pose commands; IMU is analytic specific force from that
same prescribed trajectory, independently noised, not an estimator/GT input channel.
"""
import argparse,hashlib,json,pathlib
import cv2,numpy as np,rosbag,rospy
from sensor_msgs.msg import Image,Imu,CameraInfo,PointCloud2,PointField
from std_msgs.msg import Header
from trajectory_kinematics import quat, rotation, state
Q=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
def hospital_state(t):
 p,r=state(t+.001);return np.array([-5.,-23.,0.])+Q@(p+np.array([20.,0,0])),Q@r
T0=1000.
def stamp(t):return rospy.Time.from_sec(T0+float(t))
def head(t,frame,k):return Header(seq=k,stamp=stamp(t),frame_id=frame)
def file_sha256(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()
def main(a):
 converter_sha256=file_sha256(pathlib.Path(__file__))
 raw=pathlib.Path(a.raw);m=json.loads((raw/'raw_manifest.json').read_text())
 if m['status']!='capture_complete':raise SystemExit('Raw capture incomplete')
 out=pathlib.Path(a.output);out.mkdir(parents=True,exist_ok=True)
 if (out/'sensors.bag').exists():raise SystemExit('Refuse existing frozen bag')
 duration=m['duration_requested_s']
 state_fn=hospital_state
 if m.get('trajectory_profile')=='hospital_l_v1':
  from trajectory_profiles import hospital_l_state,profile_metadata
  profile=profile_metadata()
  if profile!=m.get('trajectory_metadata'):raise SystemExit('Trajectory source/parameters changed after capture')
  state_fn=lambda t:hospital_l_state(t+.001)
 elif m.get('trajectory_profile')=='hospital_l_excited_v2':
  from trajectory_profiles_v2 import hospital_l_excited_state,profile_metadata
  profile=profile_metadata()
  if profile!=m.get('trajectory_metadata'):raise SystemExit('Trajectory source/parameters changed after capture')
  state_fn=lambda t:hospital_l_excited_state(t+.001)
 rgb_entries=m.get('rgb_images',[])
 has_rgb=bool(rgb_entries)
 if has_rgb and (len(rgb_entries)!=len(m['images']) or any(abs(r[0]-g[0])>1e-8 for r,g in zip(rgb_entries,m['images']))):
  raise SystemExit('RGB/mono acquisitions must have matching counts and stamps')
 actual_info=m.get('actual_camera_info',[])
 if not actual_info and (raw/'actual_rig_audit.json').exists():actual_info=json.loads((raw/'actual_rig_audit.json').read_text()).get('actual_camera_info',[])
 if not actual_info:raise SystemExit('Actual Gazebo CameraInfo missing; cannot freeze guessed calibration')
 K=actual_info[-1]['K'];FX,FY,CX,CY=K[0],K[4],K[2],K[5]
 rng_i=np.random.RandomState(a.seed+10);rng_l=np.random.RandomState(a.seed+20);rng_c=np.random.RandomState(a.seed+30)
 events=[]
 for k in range(int(duration*200)+1):events.append((k/200.,'imu',k))
 for k,(t,f) in enumerate(m['images']):events.append((t,'image',k))
 for k,(t,f) in enumerate(m['scans']):events.append((t,'lidar',k))
 events.sort();truth=[];counts={'imu':0,'image':0,'lidar':0};h=1e-4;clearance=[]
 dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<f4')]);fields=[PointField(n,int(dtype.fields[n][1]),PointField.FLOAT32,1) for n in dtype.names]
 with rosbag.Bag(str(out/'sensors.bag'),'w',compression=rosbag.Compression.LZ4) as bag:
  for t,kind,k in events:
   if kind=='imu':
    P,R=state_fn(t);pm,rm=state_fn(t-h);pp,rp=state_fn(t+h);acc=(pp-2*P+pm)/h**2;W=R.T@((rp-rm)/(2*h))
    omega=np.array([W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1]])*.5+rng_i.normal(0,.001,3)
    force=R.T@(acc-[0,0,-9.81])+rng_i.normal(0,.02,3)
    msg=Imu(header=head(t,'imu',k));msg.orientation_covariance[0]=-1
    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=force
    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=omega
    msg.linear_acceleration_covariance=np.diag([.0004]*3).ravel().tolist();msg.angular_velocity_covariance=np.diag([.000001]*3).ravel().tolist()
    bag.write('/doa/imu',msg,stamp(t));truth.append([T0+t,*P,*quat(R)])
   elif kind=='image':
    if has_rgb:
     bgr=cv2.imread(str(raw/rgb_entries[k][1]),cv2.IMREAD_COLOR)
     if bgr is None:raise RuntimeError('Missing true RGB capture')
     rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
     rgb=np.clip(rgb.astype(float)+rng_c.normal(0,.8,rgb.shape),0,255).astype(np.uint8)
     image=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);height,width=image.shape
     msg=Image(header=head(t,'camera_optical',k),height=height,width=width,encoding='rgb8',is_bigendian=0,step=width*3,data=np.ascontiguousarray(rgb).tobytes())
     gray=Image(header=msg.header,height=height,width=width,encoding='mono8',is_bigendian=0,step=width,data=image.tobytes())
     bag.write('/doa/camera/image_mono',gray,stamp(t))
    else:
     image=cv2.imread(str(raw/m['images'][k][1]),cv2.IMREAD_GRAYSCALE)
     if image is None:raise RuntimeError('Missing mono capture')
     image=np.clip(image.astype(float)+rng_c.normal(0,.8,image.shape),0,255).astype(np.uint8)
     height,width=image.shape;msg=Image(header=head(t,'camera_optical',k),height=height,width=width,encoding='mono8',is_bigendian=0,step=width,data=image.tobytes())
    bag.write('/doa/camera/image_raw',msg,stamp(t));info=CameraInfo(header=msg.header,height=height,width=width,distortion_model='plumb_bob',D=[0.]*5,
     K=[FX,0,CX,0,FY,CY,0,0,1],R=np.eye(3).ravel().tolist(),P=[FX,0,CX,0,0,FY,CY,0,0,0,1,0]);bag.write('/doa/camera/camera_info',info,stamp(t))
   else:
    f=np.load(raw/m['scans'][k][1]);n=int(f['count']);v=int(f['vertical_count']);ranges=f['ranges'].astype(float)
    az=np.tile(np.linspace(float(f['angle_min']),float(f['angle_max']),n),v);el=np.repeat(np.linspace(float(f['vertical_angle_min']),float(f['vertical_angle_max']),v),n)
    valid=np.isfinite(ranges)&(ranges>=.2)&(ranges<25.);clearance.append([t,float(np.min(ranges[valid])),int(valid.sum())])
    noisy=ranges[valid]+rng_l.normal(0,.02,valid.sum());directions=np.column_stack([np.cos(el[valid])*np.cos(az[valid]),np.cos(el[valid])*np.sin(az[valid]),np.sin(el[valid])]);xyz=directions*noisy[:,None]
    data=np.zeros(len(noisy),dtype=dtype)
    for j,key in enumerate(['x','y','z']):data[key]=xyz[:,j]
    # rgb packed field is constant zero; no extra camera depth is supplied.
    msg=PointCloud2(header=head(t,'lidar',k),height=1,width=len(data),fields=fields,is_bigendian=False,point_step=dtype.itemsize,row_step=dtype.itemsize*len(data),data=data.tobytes(),is_dense=True)
    bag.write('/doa/lidar',msg,stamp(t))
   counts[kind]+=1
 np.savetxt(out/'truth.tum',truth,fmt='%.9f',header='timestamp tx ty tz qx qy qz qw')
 np.savetxt(out/'scan_clearance.csv',clearance,delimiter=',',header='t,min_surface_range_m,valid_points',comments='')
 manifest={'status':'generated_not_estimator_validated','generator':'actual_gazebo_hospital_rendered_rgb_and_flash_lidar',
  'raw_capture':str(raw),'raw_manifest_sha256':hashlib.sha256((raw/'raw_manifest.json').read_bytes()).hexdigest(),
  'sensor_pose_time_lead_s':.001,'timing_policy':'raw Gazebo header t uses commanded pose at t+.001; analytic IMU/truth follow matching continuous p(t+.001), same all modes','source_world':m['source_world'],'source_world_sha256':m['source_world_sha256'],'duration_s':duration,'texture':m.get('texture','rich'),'seed':a.seed,'counts':counts,
  'body_frame':'right-handed FLU x forward y left z up','camera_frame':'right down forward','R_camera_body':[[0,-1,0],[0,0,-1],[1,0,0]],
  'camera_position_body_m':[.05,0,.03],'camera':{'width':640,'height':480,'fx':FX,'fy':FY,'cx':CX,'cy':CY,'distortion':[0]*5,
   'encoding':'rgb8' if has_rgb else 'mono8','primary_image_topic':'/doa/camera/image_raw','mono_derived_topic':'/doa/camera/image_mono' if has_rgb else None},
  'R_lidar_imu':np.eye(3).tolist(),'T_lidar_imu_m':[0,0,0],
  'lidar':{'type':'instantaneous_gazebo_gpu_flash','lidar_model_type':4,'point_time_unit':'none: true instantaneous flash; L515 curvature0','range_noise_sigma_m':.02,'scan_duration_s':0.},
  'imu':{'source':'analytic kinematics of prescribed simulator trajectory, specific force R^T(a-g), no bias','rate_hz':200,'accel_sample_sigma_mps2':.02,'gyro_sample_sigma_radps':.001},
  'truth_policy':'truth.tum separate and absent sensors.bag; only evaluator reads it','world_rotation_initial_body':Q.tolist(),'report_basis_world':Q.tolist(),
  'sensor_bag_sha256':file_sha256(out/'sensors.bag'),'converter_sha256':converter_sha256}
 manifest['trajectory_profile']=m.get('trajectory_profile','hospital_straight_v1');manifest['trajectory_metadata']=m.get('trajectory_metadata')
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--raw',required=True);ap.add_argument('--output',required=True);ap.add_argument('--seed',type=int,default=7);main(ap.parse_args())
