"""Real-process ROS shutdown regression tests, with no Gazebo or motion.

Run on a sourced Jazzy shell. Originals are only read; each case has fresh output.
Signal timing is deliberately adversarial, not a new mapping benchmark.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def wait_until(predicate, timeout=20):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    raise TimeoutError('test condition did not become ready')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root', type=Path)
    ap.add_argument('campaign', type=Path)
    args = ap.parse_args()
    root, campaign = args.root.resolve(), args.campaign.resolve()
    out = root / 'results' / 'shutdown'
    out.mkdir(exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID='176', ROS_LOCALHOST_ONLY='1',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    results = []
    started = time.monotonic()

    def case(name, source, module, original=False, seed=None):
        work = out / name
        work.mkdir()
        for p in source.glob('*.py'):
            shutil.copy2(p, work / p.name)
        data = work / 'output'
        data.mkdir()
        expected = None
        if seed:
            old = campaign / ('repeat_%02d' % seed) / 'output'
            for f in ('keyframes.json', 'tracks.json'):
                shutil.copy2(old / f, data / f)
            expected = {f: json.loads((old/f).read_text()) for f in ('keyframes.json','tracks.json')}
        # Delay final save so the second signal definitely arrives inside it.
        # The actual serializer and node main function are not replaced.
        wrapper = """
import importlib,time
from pathlib import Path
m=importlib.import_module(MODULE)
cls=m.Monocular if MODULE=='monocular' else (m.View if MODULE=='live_view' else m.WheelIMU)
init=cls.__init__
def ready(self):
 init(self);Path('READY').touch()
cls.__init__=ready
if MODULE=='monocular':
 save=cls.save
 def wrapped(self):
  Path('SAVING').touch();time.sleep(.35);save(self)
 cls.save=wrapped
m.main()
""".replace('MODULE', repr(module))
        log = (work/'process.log').open('w')
        p = subprocess.Popen([sys.executable, '-c', wrapper], cwd=work, env=env,
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        row = {'case':name,'original_expected_failure':original}
        begin = time.monotonic()
        try:
            wait_until(lambda:(work/'READY').exists() or p.poll() is not None, 30)
            if p.poll() is not None:
                raise RuntimeError('node exited before READY')
            # Give the actual ROS spin loop a chance to enter its wait set.
            time.sleep(.15)
            os.killpg(p.pid, signal.SIGINT)
            if module == 'monocular':
                wait_until(lambda:(work/'SAVING').exists() or p.poll() is not None)
                if not (work/'SAVING').exists():
                    raise RuntimeError('final save was not entered')
                for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGINT):
                    if p.poll() is None:
                        os.killpg(p.pid,sig)
                    time.sleep(.04)
            code = p.wait(timeout=35)
            log.flush()
            text = (work/'process.log').read_text()
            row.update(exit_code=code,traceback='Traceback' in text,
                       double_shutdown='rcl_shutdown already called' in text,
                       interrupted_save='KeyboardInterrupt' in text)
            if seed and not original:
                row['keyframes_equal'] = json.loads((data/'keyframes.json').read_text()) == expected['keyframes.json']
                row['tracks_equal'] = json.loads((data/'tracks.json').read_text()) == expected['tracks.json']
                row['save_complete'] = json.loads((data/'save_complete.json').read_text())
                row['temporary_files'] = [x.name for x in data.glob('*.tmp')]
            row['pass'] = ((code != 0 and row['interrupted_save']) if original and seed else
                           (code != 0 and row['double_shutdown']) if original else
                           code == 0 and not row['traceback'])
            if seed and not original:
                row['pass'] &= row['keyframes_equal'] and row['tracks_equal'] and not row['temporary_files']
        except Exception as e:
            row.update(pass_=False,error=repr(e))
            row['pass'] = False
        finally:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
            log.close()
        row['wall_seconds'] = time.monotonic()-begin
        results.append(row)
        print(json.dumps(row),flush=True)

    case('original_vision_double_signal',root/'baseline','monocular',True,1)
    case('original_viewer_shutdown',root/'baseline','live_view',True)
    for i in range(1,4):
        case('patched_vision_run_%02d'%i,root/'runtime','monocular',seed=i)
    for i in range(3):
        case('patched_viewer_%02d'%i,root/'runtime','live_view')
    case('patched_odometry',root/'runtime','odometry')

    sys.path.insert(0,str(root/'runtime'))
    from lifecycle import ProcessStopper
    for mode in ('cooperative','ignore'):
        ready = out / ('child_'+mode+'.ready')
        worker = """import signal,time
from pathlib import Path
MODE
Path(READY).touch()
LOOP
"""
        if mode=='cooperative':
            worker=worker.replace('MODE',"from lifecycle import StopRequest\ns=StopRequest();s.__enter__()")
            worker=worker.replace('LOOP',"while not s.requested:time.sleep(.01)\ntime.sleep(.1)")
        else:
            worker=worker.replace('MODE',"signal.signal(signal.SIGINT,signal.SIG_IGN)\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)")
            worker=worker.replace('LOOP',"time.sleep(30)")
        worker=worker.replace('READY',repr(str(ready)))
        p=subprocess.Popen([sys.executable,'-c',worker],cwd=root/'runtime',env=env,start_new_session=True)
        try:
            wait_until(ready.exists)
            stopper=ProcessStopper()
            first=stopper.stop(p,wait=.5,terminate_wait=.2,kill_wait=2)
            second=stopper.stop(p,wait=.5,terminate_wait=.2,kill_wait=2)
            expected=['SIGINT'] if mode=='cooperative' else ['SIGINT','SIGTERM','SIGKILL']
            results.append({'case':'owned_child_'+mode,'result':first,
                            'pass':first==second and first['signals']==expected and
                            first['exit_code']==(0 if mode=='cooperative' else -9)})
        finally:
            if p.poll() is None:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)

    report={'date_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            'platform':'Dell Vostro 3710; ROS 2 Jazzy',
            'scope':'Isolated ROS_DOMAIN_ID=176; no Gazebo, no controller or nonzero commands',
            'seconds':time.monotonic()-started,'cases':results,
            'all_pass':all(r['pass'] for r in results),
            'code_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'runtime').glob('*.py')}}
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'all_pass':report['all_pass'],'cases':len(results),'seconds':report['seconds']}),flush=True)
    if not report['all_pass']:raise SystemExit(1)


if __name__=='__main__':
    main()
