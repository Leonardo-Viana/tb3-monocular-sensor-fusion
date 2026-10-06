"""After the supervised mission ends, save a factual English report and bag audit.
Read-only with respect to robot control; no simulator geometry or pose queries.
"""
import sys,time,json,collections,hashlib,datetime
from pathlib import Path
import psutil,yaml,rosbag2_py,cv2,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output';owner=int(sys.argv[1])
while psutil.pid_exists(owner) and psutil.Process(owner).status()!='zombie':time.sleep(2)
status=json.loads((OUT/'autonomy_status.json').read_text());summary=json.loads((OUT/'stable/validation.json').read_text()) if (OUT/'stable/validation.json').exists() else {};bag_reports=[]
for bag in sorted(OUT.glob('raw*')):
 if not (bag/'metadata.yaml').exists():continue
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));counts=collections.Counter();first=last=None
 while reader.has_next():
  topic,data,stamp=reader.read_next();counts[topic]+=1
  if topic=='/camera/image_raw/compressed':
   if first is None:first=data
   last=data
 shapes=[]
 for b in [first,last]:
  m=deserialize_message(b,CompressedImage);im=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR);shapes.append(list(im.shape) if im is not None else None)
 meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];expected={t['topic_metadata']['name']:t['message_count'] for t in meta['topics_with_message_count']}
 bag_reports.append({'path':str(bag),'messages':dict(counts),'matches_metadata':dict(counts)==expected,'endpoint_rgb_shapes':shapes,'bytes':sum(p.stat().st_size for p in bag.glob('*')),'duration_sim_s':meta['duration']['nanoseconds']*1e-9,'files_sha256':{p.name:hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in bag.glob('*.mcap')}})
 report={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'post-mission readback','final_control':status,'last_stable_reconstruction':summary,'bags':bag_reports,'dense_3d_complete':False}
 (OUT/'FINAL_REPORT.json').write_text(json.dumps(report,indent=2))
rows=[]
for p in sorted((OUT/'attempts').glob('*/autonomy_status.json')):
 s=json.loads(p.read_text());rows.append(f"- Preserved attempt: {s['reason']}; {s['goals_reached']} goals; {s['travelled_m']:.3f} m; {s['elapsed_wall_s']:.1f} s.")
t=summary.get('test_pixels',{});metrics=status.get('map_metrics',{})
text=f'''# Autonomous textured-scene mapping: execution report

Generated {datetime.datetime.now(datetime.timezone.utc).isoformat()}.

## Actual terminal state

The guarded-tracker controller stopped with reason **{status['reason']}**. It reached {status['goals_reached']} goals and travelled approximately {status['travelled_m']:.3f} m according to fused map-frame pose increments. This is not ground-truth distance. It recorded {status['failed_goals']} failed goals and {status['recoveries']} limited recovery attempts. The supervisor's separate manifest records any infrastructure error.

{chr(10).join(rows)}

## Last completed reconstruction

The last completed fit contains {summary.get('accepted_landmarks',0)} accepted sparse RGB landmark IDs from {summary.get('frames',0)} keyframes. Reserved pixels: {t}. Aggregate pixel gate: {summary.get('dataset_pixel_gate_pass',False)}. Consult the numerical evidence for full tails, unscored observations and nonconverged blocks. Counts may include multiple IDs for one physical feature. Reconstruction may lag the newest acquired frame if a final job failed or timed out; the keyframe file and fit summary preserve both sizes.

Planner metrics: {json.dumps(metrics)}. Coverage refers to a finite reachable camera-view lattice, not dense surface completeness. Full 3D completion and absolute metric accuracy are **not established**.

The estimator consumed RGB image rays, wheel velocity, raw gyro/accelerometer, and 2D LiDAR pose support. No RGB-D, compass, IMU orientation field or simulator/world geometry was an estimator input. Historical datasets were preserved. This trial starts from an empty estimator state and uses the guarded tracker with unchanged reconstruction thresholds. The historical baseline is stored separately.

## Recordings

{len(bag_reports)} MCAP recording segments were traversed. All per-topic count comparisons passed: {all(b['matches_metadata'] for b in bag_reports)}. First and last RGB frames were decoded for every segment. Raw data remain at the paths and hashes in FINAL_REPORT.json; successful traversal is not proof of zero acquisition loss.

One RViz is used for the live/saved mission display. The simulator is requested to pause on supervisor termination. The command audit retains final observed velocity commands. See the audit and supervisor manifest rather than inferring robot state from this report alone.

This is one prospective development repeat with a guarded tracker, not a randomized or statistically replicated causal comparison. Closed-loop paths and runtime load can differ from the baseline. No publication or public upload was performed.
'''
(OUT/'FINAL_REPORT.md').write_text(text);print(json.dumps({'report':str(OUT/'FINAL_REPORT.md'),'bags':len(bag_reports),'reason':status['reason'],'points':summary.get('accepted_landmarks',0)}),flush=True)
