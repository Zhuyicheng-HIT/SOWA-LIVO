#!/usr/bin/env python3
"""Render actual Gazebo RGB views and flash ranges at fixed scene inspection poses."""
import argparse, hashlib, json, math, os, pathlib, signal, subprocess, sys, threading, time
import xml.etree.ElementTree as ET
import cv2,numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.world_control_pb2 import WorldControl
from google.protobuf import text_format

def main(a):
    out=pathlib.Path(a.output);out.mkdir(parents=True,exist_ok=True)
    if (out/'preview_manifest.json').exists():raise RuntimeError('Refuse existing preview')
    os.environ['GZ_PARTITION']='doa_hospital_l_preview_'+str(os.getpid())
    env=dict(os.environ,LIBGL_ALWAYS_SOFTWARE='1',MESA_LOADER_DRIVER_OVERRIDE='swrast',GALLIUM_DRIVER='llvmpipe',EGL_PLATFORM='surfaceless')
    tree=ET.parse(a.world);world=tree.getroot().find('world');name=world.get('name')
    rig=ET.fromstring('''<model name="doa_preview_rig"><static>true</static><pose>-5 -23 1.55 0 0 1.57079632679</pose><link name="sensors">
    <sensor name="rgb" type="camera"><always_on>true</always_on><update_rate>10</update_rate><topic>/doa/preview/rgb</topic><camera><horizontal_fov>1.3494818844</horizontal_fov><image><width>960</width><height>720</height><format>R8G8B8</format></image><clip><near>0.08</near><far>100</far></clip></camera></sensor>
    <sensor name="lidar" type="gpu_lidar"><always_on>true</always_on><update_rate>10</update_rate><topic>/doa/preview/lidar</topic><lidar><scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159265359</min_angle><max_angle>3.14159265359</max_angle></horizontal><vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.6</min_angle><max_angle>0.6</max_angle></vertical></scan><range><min>0.2</min><max>25</max><resolution>0.002</resolution></range></lidar></sensor>
    </link></model>''');world.append(rig)
    sdf=out/'preview_world.sdf';tree.write(sdf,encoding='utf-8',xml_declaration=True)
    lock=threading.Lock();images=[];scans=[]
    def timestamp(m):return m.header.stamp.sec+m.header.stamp.nsec*1e-9
    def image_cb(m):
        with lock:images.append((timestamp(m),m.width,m.height,m.step,bytes(m.data)))
    def scan_cb(m):
        with lock:scans.append((timestamp(m),np.array(m.ranges,dtype=np.float32)))
    node=Node();node.subscribe(Image,'/doa/preview/rgb',image_cb);node.subscribe(LaserScan,'/doa/preview/lidar',scan_cb)
    log=open(out/'gazebo.log','w');proc=subprocess.Popen(['gz','sim','-s','--headless-rendering',str(sdf)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    broker=subprocess.Popen([sys.executable,'-s',a.broker],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    def request(service,req):
        broker.stdin.write(json.dumps({'service':'/world/'+name+'/'+service,'type':req.DESCRIPTOR.full_name,'text':text_format.MessageToString(req)})+'\n');broker.stdin.flush()
        reply=json.loads(broker.stdout.readline())
        if not reply['ok']:raise RuntimeError(reply)
    views=[('01_main_entrance',[-5.,-23.,1.55,math.pi/2]),
           ('02_long_straight',[-5.,35.,1.55,math.pi/2]),
           ('03_approach_corner',[-5.,98.,1.55,math.pi/3]),
           ('04_l_corner',[-4.4,104.4,1.55,.35]),
           ('05_return_leg',[18.,105.,1.55,0.]),
           ('06_closed_doors',[-5.7,54.,1.55,2.05])]
    result=[];error=None
    try:
        for attempt in range(50):
            try:request('control',WorldControl(pause=True));break
            except Exception:
                if proc.poll() is not None:raise RuntimeError('Gazebo exited at startup')
                time.sleep(.2)
        else:raise RuntimeError('World control service timeout')
        for i,(label,(x,y,z,yaw)) in enumerate(views):
            p=Pose();p.name='doa_preview_rig';p.position.x=x;p.position.y=y;p.position.z=z
            p.orientation.z=math.sin(yaw/2);p.orientation.w=math.cos(yaw/2)
            request('set_pose',p);target=.001+i*.2
            q=WorldControl(pause=False);q.run_to_sim_time.sec=int(target);q.run_to_sim_time.nsec=round((target-int(target))*1e9)
            request('control',q);deadline=time.monotonic()+40
            while True:
                with lock:
                    ims=[v for v in images if abs(v[0]-target)<.002];ls=[v for v in scans if abs(v[0]-target)<.002]
                if ims and ls:break
                if proc.poll() is not None:raise RuntimeError('Gazebo exited during rendering')
                if time.monotonic()>deadline:raise RuntimeError(f'Missing target tick {target}; image timestamps {[v[0] for v in images]}')
                time.sleep(.01)
            t,ww,hh,step,data=ims[-1];rgb=np.frombuffer(data,np.uint8).reshape(hh,step)[:,:ww*3].reshape(hh,ww,3)
            file=label+'.png';cv2.imwrite(str(out/file),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            np.savez_compressed(out/(label+'_ranges.npz'),ranges=ls[-1][1])
            gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);corners=cv2.goodFeaturesToTrack(gray,1500,.01,8)
            result.append({'label':label,'file':file,'world_pose_xyz_yaw':[x,y,z,yaw],'sim_timestamp':t,'rgb_shape':list(rgb.shape),
                           'shi_tomasi_corners':0 if corners is None else len(corners),'rgb_sha256':hashlib.sha256((out/file).read_bytes()).hexdigest()})
            print(label,'captured',result[-1]['shi_tomasi_corners'],'corners',flush=True)
    except Exception as e:error=repr(e);raise
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        broker.terminate()
        try:broker.wait(timeout=3)
        except subprocess.TimeoutExpired:broker.kill();broker.wait()
        log.close()
        m={'status':'preview_complete' if error is None else 'preview_failed','error':error,'source_world':a.world,
           'source_world_sha256':hashlib.sha256(pathlib.Path(a.world).read_bytes()).hexdigest(),
           'capture':'Actual Gazebo Ogre2 rendered RGB8, 960x720 inspection camera; not an algorithm dataset',
           'views':result,'gazebo_process_exit':proc.returncode}
        (out/'preview_manifest.json').write_text(json.dumps(m,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--world',required=True);p.add_argument('--output',required=True);p.add_argument('--broker',required=True);main(p.parse_args())
