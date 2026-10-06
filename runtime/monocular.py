"""Original RGB camera -> tracked image rays -> multi-view 3D landmarks.

Metric poses: wheel+raw-IMU EKF -> 2-D lidar SLAM. Only RGB rays generate
3-D landmarks. No depth sensor, simulator poses, compass or wall extrusion.
"""
import os,time,json,sys
from pathlib import Path
from collections import Counter
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage,CameraInfo,PointCloud2,PointField
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header,String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer,TransformListener
from geometry import OPTICAL_TO_BASE,triangulate,graph_warp
from tracking_policy import transition_allowed

def sec(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

class Monocular(Node):
    def __init__(self):
        super().__init__('monocular_reconstruction');cv2.setNumThreads(1)
        self.out=Path(__file__).parent/'output';(self.out/'frames').mkdir(parents=True,exist_ok=True)
        self.latest=None;self.K=None;self.last_stamp=-1.;self.last_T=None
        self.old_gray=None;self.tracking_stamp=None;self.tracking_T=None;self.active={};self.tracks={};self.frames=[];self.next_id=0
        self.nodes={};self.graph_revisions=0;self.loop_edges=0
        self.count=Counter();self.reject=Counter();self.landmarks=[]
        self.resume()
        self.last_save=time.monotonic();self.last_rebuild=-1.;self.last_report=0.
        self.tf=Buffer(cache_time=rclpy.duration.Duration(seconds=120));self.listener=TransformListener(self.tf,self)
        self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.image_cb,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.info_cb,qos_profile_sensor_data)
        self.create_subscription(MarkerArray,'/mono/graph',self.graph_cb,10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(PointCloud2,'/mono/visual_points',qos)
        self.status_pub=self.create_publisher(String,'/mono/vision_status',qos)
        self.path_pub=self.create_publisher(RosPath,'/mono/camera_path',qos)
        self.preview=self.create_publisher(CompressedImage,'/mono/tracks/compressed',1)
        self.create_timer(.12,self.process)
        self.create_timer(8.,self.rebuild)
        self.create_service(Trigger,'/mono/save_visual',self.save_cb)
        self.get_logger().info('MONOCULAR ONLY. 3D from RGB triangulation; no depth or compass subscriptions.')

    def resume(self):
        kp=self.out/'keyframes.json';tp=self.out/'tracks.json'
        if not kp.exists() or not tp.exists():return
        d=json.loads(kp.read_text());self.K=np.array(d['K'])
        self.frames=[{'T':np.array(f['T']),'stamp':f['stamp'],'anchors':{int(k):np.array(v) for k,v in f['anchors'].items()}} for f in d['frames']]
        self.nodes={int(k):np.array(v) for k,v in d['graph'].items()}
        self.tracks={int(k):{'obs':[(int(f),np.array(p,dtype=np.float32)) for f,p in t['obs']],'rgb':np.array(t['rgb'],dtype=np.uint8)} for k,t in json.loads(tp.read_text()).items()}
        self.next_id=max(self.tracks,default=-1)+1
        self.count['resumed_frames']=len(self.frames)
        self.get_logger().info(f'Resumed {len(self.frames)} frames and {len(self.tracks)} tracks; first new image starts fresh temporal tracks.')

    def image_cb(self,m):self.latest=m;self.count['rgb_messages']+=1
    def info_cb(self,m):
        self.K=np.array(m.k).reshape(3,3).copy();self.K[:2]*=960./m.width

    def graph_cb(self,m):
        updated={int(p.id):np.array([p.pose.position.x,p.pose.position.y]) for p in m.markers if p.type==2 and p.header.frame_id=='mono_map'}
        if updated:
            if any(k in self.nodes and np.linalg.norm(v-self.nodes[k])>.005 for k,v in updated.items()):self.graph_revisions+=1
            self.nodes.update(updated)
        # Marker id 1 is recorded as a diagnostic only; topology is saved for audit.
        self.loop_edges=max([len(p.points)//2 for p in m.markers if p.type==5 and p.id==1]+[self.loop_edges])

    def anchors(self,T):
        ids=sorted(self.nodes,key=lambda k:np.linalg.norm(self.nodes[k]-T[:2,3]))[:8]
        return {k:self.nodes[k].copy() for k in ids if np.linalg.norm(self.nodes[k]-T[:2,3])<1.2}

    def process(self):
        m=self.latest
        if m is None or self.K is None or sec(m)-self.last_stamp<.32:return
        try:
            tf=self.tf.lookup_transform('mono_map','mono_camera_link',rclpy.time.Time.from_msg(m.header.stamp))
        except Exception:
            self.count['tf_wait']+=1;return
        tr=tf.transform;q=tr.rotation
        T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()@OPTICAL_TO_BASE
        T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
        self.last_stamp=sec(m)
        if self.last_T is not None:
            displacement=np.linalg.norm(T[:3,3]-self.last_T[:3,3])
            angle=Rotation.from_matrix(self.last_T[:3,:3].T@T[:3,:3]).magnitude()
            if displacement<.008 and angle<.008:return
        if len(self.frames)>=5500:
            self.count['frame_capacity']+=1;return
        im=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
        if im is None:return
        im=cv2.resize(im,(960,540));gray=cv2.GaussianBlur(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),(5,5),.9)
        fid=len(self.frames);self.frames.append({'T':T,'stamp':sec(m),'anchors':self.anchors(T)})
        self.count['processed_frames']+=1;self.last_T=T.copy()
        cv2.imwrite(str(self.out/'frames'/f'{fid:05d}.jpg'),im,[cv2.IMWRITE_JPEG_QUALITY,82])
        surviving={}
        if self.old_gray is not None and self.tracking_stamp is not None:
            allowed,why=transition_allowed(self.tracking_stamp,sec(m),self.tracking_T,T)
            if not allowed:
                self.active={};self.old_gray=None
                self.count['tracking_reset_'+why]+=1
        if self.old_gray is not None and self.active:
            ids=list(self.active);old=np.array([self.active[k] for k in ids],np.float32).reshape(-1,1,2)
            new,status,err=cv2.calcOpticalFlowPyrLK(self.old_gray,gray,old,None,winSize=(25,25),maxLevel=4,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.01))
            back,sback,_=cv2.calcOpticalFlowPyrLK(gray,self.old_gray,new,None,winSize=(25,25),maxLevel=4,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.01))
            fb=np.linalg.norm(back-old,axis=2).ravel();xy=new.reshape(-1,2)
            ok=status.ravel().astype(bool)&sback.ravel().astype(bool)&(fb<.65)&(err.ravel()<12)
            ok&=(xy[:,0]>12)&(xy[:,0]<948)&(xy[:,1]>12)&(xy[:,1]<528)
            for k,p,valid in zip(ids,xy,ok):
                if valid:
                    surviving[k]=p;self.tracks[k]['obs'].append((fid,p.copy()))
                    if len(self.tracks[k]['obs'])>24:self.tracks[k]['obs'].pop(1)
                    self.count['tracked_observations']+=1
        self.active=surviving
        mask=np.full(gray.shape,255,np.uint8);mask[:12]=0;mask[-12:]=0;mask[:,:12]=0;mask[:,-12:]=0
        for p in self.active.values():cv2.circle(mask,tuple(np.round(p).astype(int)),12,0,-1)
        pts=cv2.goodFeaturesToTrack(gray,maxCorners=max(1,450-len(self.active)),qualityLevel=.015,minDistance=12,mask=mask,blockSize=7)
        eig=cv2.cornerMinEigenVal(gray,blockSize=7)
        if pts is not None:
            for p in pts.reshape(-1,2):
                x,y=np.round(p).astype(int)
                if eig[y,x]<.00015:continue
                k=self.next_id;self.next_id+=1
                self.active[k]=p;self.tracks[k]={'obs':[(fid,p.copy())],'rgb':im[y,x,::-1].copy()}
                self.count['features_started']+=1
        # Discard dead two-view/no-baseline tracks to keep memory bounded.
        for k in list(self.tracks):
            if k not in self.active and len(self.tracks[k]['obs'])<3:del self.tracks[k]
        preview=im.copy()
        for p in self.active.values():cv2.circle(preview,tuple(p.astype(int)),3,(0,220,255),1)
        cv2.putText(preview,f'RGB monocular | tracks {len(self.active)} | 3D {len(self.landmarks)}',(15,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,200,255),2)
        cv2.imwrite(str(self.out/'tracks_latest.jpg'),preview)
        ok,jpeg=cv2.imencode('.jpg',preview)
        if ok:
            pm=CompressedImage();pm.header=m.header;pm.format='jpeg';pm.data=jpeg.tobytes();self.preview.publish(pm)
        self.old_gray=gray;self.tracking_stamp=sec(m);self.tracking_T=T.copy()

    def corrected(self):
        return [graph_warp(f['T'],f['anchors'],self.nodes) for f in self.frames]

    def rebuild(self):
        if not self.frames:
            self.status_pub.publish(String(data=json.dumps({'frames':0,'counts':dict(self.count)})));return
        poses=self.corrected();landmarks=[];rejections=Counter()
        for k,track in self.tracks.items():
            obs=track['obs']
            if len(obs)>8:obs=[obs[i] for i in np.linspace(0,len(obs)-1,8).astype(int)]
            result,why=triangulate([p for f,p in obs],[poses[f] for f,p in obs],self.K)
            if result is not None:
                result.update(track_id=k,rgb=track['rgb'],observations=len(track['obs']))
                landmarks.append(result)
            else:rejections[why]+=1
        self.landmarks=landmarks;self.reject=rejections
        msg=PointCloud2();msg.header=Header(stamp=self.get_clock().now().to_msg(),frame_id='mono_map')
        dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]
        a=np.zeros(len(landmarks),dtype=dtype)
        for i,p in enumerate(landmarks):
            a[i]['x'],a[i]['y'],a[i]['z']=p['xyz']
            c=p['rgb'].astype(np.uint32);a[i]['rgb']=(c[0]<<16)|(c[1]<<8)|c[2]
        msg.height=1;msg.width=len(a);msg.fields=[PointField(name=n,offset=j*4,datatype=PointField.FLOAT32 if j<3 else PointField.UINT32,count=1) for j,n in enumerate(['x','y','z','rgb'])]
        msg.point_step=16;msg.row_step=16*len(a);msg.is_dense=True;msg.data=a.tobytes();self.pub.publish(msg)
        path=RosPath();path.header=msg.header
        for T in poses:
            p=PoseStamped();p.header=msg.header;p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,T[:3,3]);p.pose.orientation.w=1.;path.poses.append(p)
        self.path_pub.publish(path)
        s=self.status();self.status_pub.publish(String(data=json.dumps(s)))
        (self.out/'vision_status.json').write_text(json.dumps(s,indent=2))
        if time.monotonic()-self.last_report>20:self.get_logger().info(json.dumps(s));self.last_report=time.monotonic()
        if time.monotonic()-self.last_save>20:self.save();self.last_save=time.monotonic()

    def status(self):
        X=np.array([p['xyz'] for p in self.landmarks])
        return {'source':'original monocular RGB only; triangulated image tracks',
                'pose_sources':['wheel velocity','raw gyro','raw accelerometer','2D lidar SLAM'],
                'forbidden_inputs_used':[],'frames':len(self.frames),'active_tracks':len(self.active),
                'visual_landmarks':len(self.landmarks),'non_ground_landmarks':int((X[:,2]>.15).sum()) if len(X) else 0,
                'height_range_m':[float(X[:,2].min()),float(X[:,2].max())] if len(X) else [],
                'median_reprojection_px':float(np.median([p['reprojection_px'] for p in self.landmarks])) if self.landmarks else None,
                'median_parallax_deg':float(np.median([p['parallax_deg'] for p in self.landmarks])) if self.landmarks else None,
                'graph_nodes':len(self.nodes),'graph_revisions':self.graph_revisions,'loop_edge_marker_count':self.loop_edges,
                'rejected_tracks':dict(self.reject),'counts':dict(self.count)}

    def save(self):
        (self.out/'vision_status.json').write_text(json.dumps(self.status(),indent=2))
        serial=[]
        for f in self.frames:serial.append({'T':f['T'].tolist(),'stamp':f['stamp'],'anchors':{str(k):v.tolist() for k,v in f['anchors'].items()}})
        (self.out/'keyframes.json').write_text(json.dumps({'K':self.K.tolist() if self.K is not None else None,'frames':serial,'graph':{str(k):v.tolist() for k,v in self.nodes.items()}}))
        tracks={str(k):{'obs':[(f,p.tolist()) for f,p in t['obs']],'rgb':t['rgb'].tolist()} for k,t in self.tracks.items()}
        (self.out/'tracks.json').write_text(json.dumps(tracks))
        with (self.out/'monocular_points.ply').open('w') as f:
            f.write('ply\nformat ascii 1.0\nelement vertex '+str(len(self.landmarks))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            for p in self.landmarks:f.write(' '.join(map(str,[*p['xyz'],*p['rgb']]))+'\n')
        (self.out/'landmark_evidence.json').write_text(json.dumps([{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in p.items()} for p in self.landmarks],indent=2))

    def save_cb(self,req,res):self.rebuild();self.save();res.success=True;res.message=str(self.out);return res

def main():
    rclpy.init();n=Monocular()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.save();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
