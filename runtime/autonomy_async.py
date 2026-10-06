"""Scheduling amendment: refine immutable snapshots while exploration continues.
Uses the same fitted-block code and thresholds. Historical synchronous attempt retained.
"""
import json,time,os,signal,subprocess,shutil
from pathlib import Path
import autonomy as base
from std_srvs.srv import Trigger
ROOT=Path(__file__).resolve().parent
class AsyncAutonomous(base.Autonomous):
 def __init__(self,seconds):
  super().__init__(seconds);self.job_started=None;self.job_snapshot=None;self.job_count=0
  h=self.out/'view_history.json'
  if h.exists():self.views.extend(json.loads(h.read_text()))
 def finish_job(self):
  if self.child is None or self.child.poll() is None:return
  code=self.child.returncode;self.child=None
  if code==0:
   s=json.loads((self.out/'stable/validation.json').read_text());ids=self.landmark_ids();gain=len(ids-self.known_track_ids)/max(1,len(self.known_track_ids));self.known_track_ids.update(ids);self.gains.append(gain)
   self.checkpoints.append({'snapshot':str(self.job_snapshot),'accepted_landmarks':s['accepted_landmarks'],'test_pixels':s['test_pixels'],'dataset_pixel_gate_pass':s['dataset_pixel_gate_pass'],'fit_seconds':s['duration_s']})
  else:self.checkpoints.append({'snapshot':str(self.job_snapshot),'fit_exit_code':code,'retained_previous_map':True})
  base.atomic(self.out/'async_jobs.json',self.checkpoints)
 def healthy(self):
  self.finish_job()
  if self.child is not None and (time.monotonic()-self.job_started>900 or base.available_mb()<400):
   os.killpg(self.child.pid,signal.SIGTERM);self.child.wait(timeout=5);self.finish_job()
  return super().healthy()
 def save_snapshot(self):
  if not self.saver.service_is_ready():self.reason='save_service_missing';return None
  self.send();future=self.saver.call_async(Trigger.Request());deadline=time.monotonic()+45
  while not future.done() and time.monotonic()<deadline and not self.stopped:self.spin_stopped(.05)
  if not future.done() or not future.result().success:self.reason='save_failed';return None
  self.job_count+=1;snap=self.out/'snapshots'/f'async_{os.getpid()}_{self.job_count:03d}';snap.mkdir(parents=True)
  for name in ['keyframes.json','tracks.json']:shutil.copy2(self.out/name,snap/name)
  return snap
 def start_job(self,snapshot):
  self.job_snapshot=snapshot;self.job_started=time.monotonic()
  self.child=subprocess.Popen(['python3',str(ROOT/'stable_cached.py'),str(snapshot),str(self.out/'stable')],env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'),stdout=open(self.out/'async_refine.log','a'),stderr=subprocess.STDOUT,start_new_session=True)
 def wait_job(self,deadline):
  while self.child is not None and self.child.poll() is None and time.monotonic()<deadline and not self.stopped:
   self.spin_stopped(.1)
   if base.available_mb()<400:break
  if self.child is not None and self.child.poll() is None:
   os.killpg(self.child.pid,signal.SIGTERM);self.child.wait(timeout=5)
  self.finish_job()
 def checkpoint(self,final=False):
  self.finish_job();self.last_checkpoint_goal=len(self.goals)
  if not final and self.child is not None:return True
  self.phase='saving_and_refining';self.send();self.status(time.monotonic()-self.session_start)
  if final and self.child is not None:self.wait_job(time.monotonic()+600)
  if self.stopped:return False
  if base.available_mb()<600:self.reason='memory_insufficient_for_refinement';return False
  snapshot=self.save_snapshot()
  if snapshot is None:return False
  self.start_job(snapshot)
  if final:self.wait_job(time.monotonic()+900)
  self.phase='exploring';return True
if __name__=='__main__':
 base.Autonomous=AsyncAutonomous;base.main()
