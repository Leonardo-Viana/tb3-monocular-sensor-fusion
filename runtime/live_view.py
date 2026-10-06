"""One live RViz view; saved stable submaps are reanchored rigidly to the live graph."""
import json,time
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Header,String
from geometry import graph_warp
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output'
class View(Node):
 def __init__(self):
  super().__init__('monocular_mission_view');q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.cloud=self.create_publisher(PointCloud2,'/mission/points',q);self.outline=self.create_publisher(PointCloud2,'/mission/lidar_outline',q);self.labels=self.create_publisher(Marker,'/mission/state',q)
  self.graph={};self.pts=[];self.summary={};self.anchors={};self.generation=None;self.provisional=None;self.map=None;self.status={}
  self.create_subscription(PointCloud2,'/mono/visual_points',lambda m:setattr(self,'provisional',m),q)
  self.create_subscription(OccupancyGrid,'/mono/map',lambda m:setattr(self,'map',m),q)
  self.create_subscription(MarkerArray,'/mono/graph',self.graph_cb,10);self.create_subscription(String,'/mono/autonomy_status',self.status_cb,10);self.create_timer(2.,self.publish)
 def status_cb(self,m):
  try:self.status=json.loads(m.data)
  except ValueError:pass
 def graph_cb(self,m):
  self.graph.update({int(p.id):np.array([p.pose.position.x,p.pose.position.y]) for p in m.markers if p.type==2 and p.header.frame_id=='mono_map'})
 def publish(self):
  h=Header(stamp=self.get_clock().now().to_msg(),frame_id='mono_map');marker=OUT/'stable/generation.json'
  if marker.exists():
   try:
    key=marker.read_text()
    if key!=self.generation:
     pts=json.loads((OUT/'stable/stable_landmarks.json').read_text());summary=json.loads((OUT/'stable/validation.json').read_text());anchors=json.loads((OUT/'stable/anchors.json').read_text())
     if key==marker.read_text():self.pts=pts;self.summary=summary;self.anchors=anchors;self.generation=key
   except (OSError,ValueError,KeyError):pass
  if self.generation:
   graph=dict(self.anchors['graph']);graph.update({str(k):v for k,v in self.graph.items()});graph={int(k):np.array(v) for k,v in graph.items()}
   transforms={int(k):graph_warp(np.eye(4),{int(i):np.array(v) for i,v in a.items()},graph) for k,a in self.anchors['frames'].items()}
   a=np.zeros(len(self.pts),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
   for i,p in enumerate(self.pts):
    xyz=(transforms[p['anchor_frame']]@np.r_[p['local_xyz'],1.])[:3];a[i]['x'],a[i]['y'],a[i]['z']=xyz;r,g,b=p['rgb'];a[i]['rgb']=(int(r)<<16)|(int(g)<<8)|int(b)
   m=PointCloud2();m.header=h;m.height=1;m.width=len(a);m.point_step=16;m.row_step=16*len(a);m.is_dense=True;m.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,n in enumerate(['x','y','z','rgb'])];m.data=a.tobytes();self.cloud.publish(m)
   note=f"Submapas RGB: {len(a)} pontos | teste agregado: {'passou' if self.summary['dataset_pixel_gate_pass'] else 'NAO passou'}"
  else:
   if self.provisional is not None:self.cloud.publish(self.provisional)
   note='RGB provisorio: aguardando primeiro refinamento'
  if self.map is not None:
   d=self.map;yy,xx=np.where(np.array(d.data).reshape(d.info.height,d.info.width)>=65);a=np.zeros((len(xx),3),np.float32);a[:,0]=(xx+.5)*d.info.resolution+d.info.origin.position.x;a[:,1]=(yy+.5)*d.info.resolution+d.info.origin.position.y
   m=PointCloud2();m.header=h;m.height=1;m.width=len(a);m.point_step=12;m.row_step=12*len(a);m.is_dense=True;m.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])];m.data=a.astype('<f4').tobytes();self.outline.publish(m)
   p=d.info.origin;mdata={'frame_id':d.header.frame_id,'resolution':d.info.resolution,'width':d.info.width,'height':d.info.height,'position':[p.position.x,p.position.y,p.position.z],'quaternion':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],'data':list(d.data)}
   tmp=OUT/'map_snapshot.tmp';tmp.write_text(json.dumps(mdata));tmp.replace(OUT/'map_snapshot.json')
  s=self.status;states={'waiting_sensors':'Aguardando sensores','frontier':'Explorando fronteiras','visual_view':'Ampliando vistas RGB','saving_and_refining':'Parado: refinando 3D','returning':'Retornando','stopped':'Missao encerrada','limited_reverse':'Recuperacao curta'};state=states.get(s.get('state'),s.get('state','Preparando missao'))
  label=Marker();label.header=h;label.ns='mission';label.id=0;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.position.x=2.;label.pose.position.y=-2.;label.pose.position.z=1.3;label.pose.orientation.w=1.;label.scale.z=.13;label.color.r=.05;label.color.g=.08;label.color.b=.12;label.color.a=1.
  label.text=f"{state}\n{note}\nVistas navegaveis: {100*s.get('map_metrics',{}).get('view_coverage',0):.0f}% | metas: {s.get('goals_reached',0)}\n3D esparso; cobertura de vistas nao prova mapa 3D completo"
  if s.get('state')=='stopped':label.text+='\n'+s.get('reason','')
  self.labels.publish(label)
def main():
 rclpy.init();n=View()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
