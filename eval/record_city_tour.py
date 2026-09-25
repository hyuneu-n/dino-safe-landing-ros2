#!/usr/bin/env python3
"""Render a scripted environment flythrough from a dedicated Gazebo RGB camera.
This is a camera tour, not autonomous flight or obstacle-avoidance evidence.
Frames wait for the pose service and fresh camera stamps before H.264 encoding.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

# time, camera position, look-at position, chapter
KEYS=[
 (0,(-305,-170,75),(-220,-55,8),'LOGISTICS / DEPARTURE'),
 (5,(-185,-150,115),(-65,0,25),'DOWNTOWN'),
 (10,(5,-130,110),(100,0,25),'DOWNTOWN'),
 (15,(160,-30,95),(230,50,10),'SHOPS / INDUSTRIAL'),
 (20,(370,-115,110),(480,25,12),'RESIDENTIAL TOWN'),
 (25,(650,-160,140),(735,-420,25),'RIVER / ELEVATED HIGHWAY'),
 (31,(910,-530,165),(735,-420,28),'BRIDGE / LOOP RAMP'),
 (37,(450,-660,190),(380,-350,20),'SOUTH BANK'),
 (43,(-300,-480,220),(-300,-40,20),'OLD TOWN'),
 (49,(-650,110,200),(-360,160,25),'HILLSIDE APPROACH'),
 (55,(180,465,150),(340,335,27),'SEOKYEONG UNIVERSITY'),
 (61,(430,230,115),(335,335,28),'SEOKYEONG UNIVERSITY'),
 (67,(770,400,205),(790,650,85),'MOUNTAIN NEIGHBORHOOD'),
 (72,(1060,800,350),(470,220,35),'CONNECTED METROPOLIS'),
 (78,(430,-720,520),(220,100,20),'CONNECTED METROPOLIS'),
]


def sample(t):
    i=next((i for i in range(len(KEYS)-1) if t<=KEYS[i+1][0]),len(KEYS)-2)
    t0,p1,q1,label=KEYS[i];t1,p2,q2,_=KEYS[i+1]
    u=max(0,min(1,(t-t0)/(t1-t0)))
    def curve(k):
        a,b,c,d=[KEYS[j][k] for j in [max(0,i-1),i,i+1,min(len(KEYS)-1,i+2)]]
        return tuple(.5*(2*b[n]+(-a[n]+c[n])*u+(2*a[n]-5*b[n]+4*c[n]-d[n])*u*u+(-a[n]+3*b[n]-3*c[n]+d[n])*u**3) for n in range(3))
    return curve(1),curve(2),label


def prepare(source,out):
    root=ET.parse(source);world=root.getroot().find('world')
    # The tour needs one camera, not six delivery-drone sensors.
    for model in list(world.findall('model')):
        if model.get('name')=='delivery_drone':world.remove(model)
    world.append(ET.fromstring('''<model name="tour_camera"><pose>-305 -170 75 0 0 0</pose><static>false</static>
      <link name="camera_link"><gravity>false</gravity><kinematic>true</kinematic>
      <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      <sensor name="rgb" type="camera"><always_on>true</always_on><update_rate>30</update_rate>
      <camera><horizontal_fov>1.30</horizontal_fov><image><width>1280</width><height>720</height><format>R8G8B8</format></image>
      <clip><near>0.2</near><far>3500</far></clip></camera>
      <plugin name="tour_rgb" filename="libgazebo_ros_camera.so"><ros><namespace>/tour</namespace></ros>
      <camera_name>rgb</camera_name><frame_name>tour_camera_link</frame_name></plugin></sensor></link></model>'''))
    root.write(out,encoding='unicode',xml_declaration=True)


def record(out,fps,seconds):
    import numpy as np
    from PIL import Image,ImageDraw,ImageFont
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image as RosImage
    from gazebo_msgs.srv import SetEntityState
    rclpy.init();node=Node('city_tour_recorder')
    client=node.create_client(SetEntityState,'/gazebo/set_entity_state')
    if not client.wait_for_service(timeout_sec=45):raise RuntimeError('Gazebo service not ready')
    latest=[None];serial=[0]
    def receive(msg):latest[0]=msg;serial[0]+=1
    sub=node.create_subscription(RosImage,'/tour/rgb/image_raw',receive,qos_profile_sensor_data)
    def until(test,timeout=15):
        end=time.monotonic()+timeout
        while not test():
            if time.monotonic()>end:raise TimeoutError('Fresh tour camera frame/service timeout')
            rclpy.spin_once(node,timeout_sec=.05)
    until(lambda:latest[0] is not None,60)
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(fps),'-i','-',
                         '-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',21)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    records=[];count=round(seconds*fps)
    try:
        for frame in range(count):
            t=KEYS[-1][0]*frame/max(1,count-1);pos,look,chapter=sample(t)
            dx,dy,dz=[look[k]-pos[k] for k in range(3)]
            yaw=math.atan2(dy,dx);pitch=math.atan2(-dz,math.hypot(dx,dy))
            req=SetEntityState.Request();req.state.name='tour_camera';req.state.reference_frame='world'
            req.state.pose.position.x,req.state.pose.position.y,req.state.pose.position.z=map(float,pos)
            q=req.state.pose.orientation
            q.x=-math.sin(yaw/2)*math.sin(pitch/2);q.y=math.cos(yaw/2)*math.sin(pitch/2)
            q.z=math.sin(yaw/2)*math.cos(pitch/2);q.w=math.cos(yaw/2)*math.cos(pitch/2)
            future=client.call_async(req);until(future.done)
            if not future.result().success:raise RuntimeError(f'Camera pose rejected at frame {frame}')
            baseline=serial[0];stamp=latest[0].header.stamp
            # Flush an in-flight render and accept only a newer stamped image.
            until(lambda:serial[0]>=baseline+3 and latest[0].header.stamp!=stamp)
            msg=latest[0]
            if msg.encoding!='rgb8':raise RuntimeError('Unexpected camera encoding: '+msg.encoding)
            arr=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            image=Image.fromarray(arr);draw=ImageDraw.Draw(image)
            draw.rectangle((0,658,1280,720),fill=(17,24,29))
            draw.text((26,667),chapter,font=font,fill=(243,220,155))
            draw.text((26,696),'GAZEBO CLASSIC  |  SCRIPTED CAMERA TOUR  |  NOT AUTONOMOUS FLIGHT',font=small,fill=(179,190,198))
            draw.rectangle((0,655,int(1280*(frame+1)/count),658),fill=(224,171,72))
            ff.stdin.write(image.tobytes())
            if frame%max(1,int(fps*4))==0:
                image.save(out.parent/f'tour_preview_{frame:04d}.jpg')
                print(f'[tour] {frame}/{count} {chapter}',flush=True)
            records.append(dict(frame=frame,time=frame/fps,position=pos,look_at=look,chapter=chapter))
        ff.stdin.close()
        if ff.wait(timeout=60):raise RuntimeError('ffmpeg failed')
        out.with_suffix('.json').write_text(json.dumps(dict(kind='scripted_camera_tour',autonomous_flight=False,fps=fps,frames=records),indent=2))
    finally:
        if ff.poll() is None:ff.terminate();ff.wait(timeout=15)
        node.destroy_node();rclpy.shutdown()
    print(f'[ok] {out} ({seconds}s, {fps}fps, 1280x720)',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare',type=Path);p.add_argument('--out',required=True,type=Path)
    p.add_argument('--fps',type=int,default=24);p.add_argument('--seconds',type=float,default=78)
    a=p.parse_args()
    if a.prepare:prepare(a.prepare,a.out)
    else:record(a.out,a.fps,a.seconds)
