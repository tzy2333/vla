"""Jetson UI with real acquisition and an explicit, audited operator reset."""
import argparse
import atexit
import fcntl
import json
import os
import re
from pathlib import Path
import secrets
import signal
import socket
import threading
import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from collector.engine import Collector
from collector.adapters import CAMERAS
from collector.platform_management import PlatformManagement
from collector.segments import SegmentAnnotations
from collector.review_workflow import ReviewWorkflow
from collector.camera_inventory import CameraInventory

ROOT=Path(__file__).resolve().parent


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--config',default=str(ROOT/'config.json'))
    args=ap.parse_args()
    config=json.loads(Path(args.config).read_text(encoding='utf-8'))
    # Single process owns devices/data. Recovery never runs against another live writer.
    lock=(ROOT/'.collector.lock').open('a+')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('A collector process already owns this project.')
    token=secrets.token_urlsafe(24)
    app=Collector(config,ROOT)
    management=PlatformManagement(app,ROOT,args.config)
    segments=SegmentAnnotations(app)
    reviews=ReviewWorkflow(app)
    inventory=CameraInventory(app,management)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,fmt,*args):
            if '/api/status' not in str(args) and '/preview/' not in str(args):
                print(self.log_date_time_string(),fmt%args,flush=True)

        def allowed_host(self):
            host=self.headers.get('Host','').split(':')[0]
            return host in config['allowed_hosts']

        def reply(self,body,status=200,content_type='application/json; charset=utf-8'):
            if not isinstance(body,bytes): body=json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type',content_type); self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store'); self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass

        def do_GET(self):
            if not self.allowed_host(): return self.reply({'error':'Host is not allowed.'},403)
            u=urlparse(self.path); q=parse_qs(u.query); path=u.path
            try:
                if path=='/': return self.reply((ROOT/'web/index.html').read_bytes(),content_type='text/html; charset=utf-8')
                if path=='/training-guide':
                    content=html.escape((ROOT/'training/README_PI05.md').read_text(encoding='utf-8'))
                    page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>π0.5训练接入说明</title><style>body{font-family:system-ui,Microsoft YaHei;max-width:1000px;margin:40px auto;padding:0 24px;color:#183345;line-height:1.8}pre{white-space:pre-wrap;font:inherit}</style><a href="/">返回采集界面</a><pre>'+content+'</pre></html>'
                    return self.reply(page.encode(),content_type='text/html; charset=utf-8')
                if path=='/api/session': return self.reply({'token':token,'mock':False})
                if path=='/api/status': return self.reply(app.status())
                if path=='/api/capabilities': return self.reply(app.capabilities())
                if path=='/api/robot/state':
                    snapshot=app.robot.live_snapshot()
                    packet=snapshot['packet']
                    arms={}
                    if packet and 'joint_motor_ids' in packet:
                        for side,offset in [('left',0),('right',7)]:
                            arms[side]={'motor_ids':packet['joint_motor_ids'][side],
                                'joint_angles_rad':packet['joint_angles_rad'][side],
                                'position_m':None, 'quaternion_wxyz':None}
                    return self.reply({'valid':packet is not None,'source':app.descriptor['source'],
                        'descriptor':app.descriptor,'state':app.status()['state'],
                        'received_ns':packet['received_ns'] if packet else None,
                        'reference_frame':'torso_shoulder_midpoint','arms':arms,
                        'pose_source':'deferred_offline_CAD_FK','fk_computation':'offline_only',
                        'health':snapshot['health']})
                if path=='/api/timing/diagnostics': return self.reply(app.timing_diagnostics())
                if path=='/api/episodes': return self.reply({'episodes':app.episodes()})
                if path=='/api/segments': return self.reply(segments.read(q.get('episode',[''])[0]))
                if path=='/api/reviews': return self.reply(reviews.snapshot(q.get('day',[''])[0],q.get('task_id',[''])[0]))
                if path=='/api/reviews/detail': return self.reply(reviews.read(q.get('episode',[''])[0]))
                if path=='/api/cameras/inventory': return self.reply(inventory.view())
                if path=='/api/tasks':
                    result=management.tasks()
                    for task in result['tasks']:
                        summary=reviews.snapshot(task_id=task['task_id'])['summary']
                        task['metrics'].update(pending=summary['pending'],kept=summary['final_delivery'],task_success_rate=summary['task_success_rate'],successes=summary['outcomes'].get('success',0),labeled=summary['outcomes'].get('success',0)+summary['outcomes'].get('failure',0))
                    return self.reply(result)
                if path=='/api/dashboard': return self.reply(management.dashboard(q.get('day',[''])[0],q.get('task_id',[''])[0]))
                if path=='/api/quality/thresholds': return self.reply(management.quality.thresholds())
                if path=='/api/quality/report': return self.reply(management.pipeline_report(q.get('episode',[''])[0]))
                if path=='/api/settings': return self.reply(management.settings())
                if path=='/api/robot/control-history': return self.reply(management.control_history())
                if path=='/api/camera/profiles':
                    head=getattr(app.cameras,'head',None)
                    return self.reply({'profiles':head.supported_profiles() if head else []})
                if path.startswith('/preview/'):
                    key=path.split('/')[-1].removesuffix('.jpg'); frame=app.cameras.latest(key)
                    if not frame or not app.cameras.health_check().get(key,{}).get('healthy'):
                        return self.reply({'error':'Camera frame is unavailable or stale'},503)
                    return self.reply(frame.jpeg,content_type='image/jpeg')
                if path=='/api/replay':
                    return self.reply(app.replay(q.get('episode',[''])[0],int(q.get('frame',['0'])[0])))
                if path=='/api/frame':
                    key=q.get('camera',[''])[0]
                    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',key): raise ValueError('Unknown camera.')
                    ep=q.get('episode',[''])[0]; index=int(q.get('frame',['0'])[0])
                    m=json.loads((app.episode_path(ep)/'manifest.json').read_text())
                    if key not in m.get('camera_keys',CAMERAS): raise ValueError('Camera was not recorded.')
                    if not 0<=index<m['frame_count']: raise ValueError('Frame out of range.')
                    data=(app.episode_path(ep)/'cameras'/key/f'{index:06d}.jpg').read_bytes()
                    return self.reply(data,content_type='image/jpeg')
                return self.reply({'error':'Not found'},404)
            except (ValueError,KeyError) as e: return self.reply({'error':str(e)},400)
            except Exception as e: return self.reply({'error':str(e)},500)

        def do_POST(self):
            if not self.allowed_host(): return self.reply({'error':'Host is not allowed.'},403)
            origin=self.headers.get('Origin')
            if origin and urlparse(origin).netloc!=self.headers.get('Host'):
                return self.reply({'error':'Cross-origin writes are not accepted.'},403)
            if self.headers.get('X-Collector-Token')!=token:
                return self.reply({'error':'Missing collector session token.'},403)
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=16384: raise ValueError('Invalid request size.')
                body=json.loads(self.rfile.read(length))
                path=urlparse(self.path).path
                if path=='/api/start': result=management.start_capture(body)
                elif path=='/api/reviews/acknowledge-fault':
                    with app.lock:
                        if app.phase!='FAULT' or body.get('episode_id')!=app.last_episode or (app.writer and not app.writer.queue.empty()): raise ValueError('请核对当前故障片段并等待落盘。')
                        app.phase='IDLE'; app.last_error=None
                        result={'retained':body['episode_id'],'phase':'IDLE'}
                elif path=='/api/reviews/decision': result=reviews.mutate(body)
                elif path=='/api/reviews/check': result=reviews.queue_check(body.get('episode_id'))
                elif path=='/api/reviews/targets': result=reviews.save_targets(body)
                elif path=='/api/reviews/export': result=reviews.start_export(body)
                elif path=='/api/cameras/discover': result=inventory.refresh()
                elif path=='/api/cameras/bind': result=inventory.bind(body)
                elif path=='/api/tasks/save': result=management.save_task(body)
                elif path=='/api/quality/thresholds': result=management.quality.save_thresholds(body)
                elif path=='/api/quality/report': result=management.pipeline_report(body.get('episode_id'),body)
                elif path=='/api/settings/save': result=management.save_settings(body)
                elif path=='/api/settings/apply': result=management.apply_settings(body)
                elif path=='/api/robot/reset': result=management.reset_robot(body)
                elif path=='/api/cameras/reconnect': result=app.reconnect_cameras(body.get('camera'))
                elif path=='/api/cameras/release': result=app.release_cameras(body.get('camera'))
                elif path=='/api/robot/reconnect': result=app.reconnect_robot()
                elif path=='/api/stop': result=app.stop()
                elif path=='/api/segments': result=segments.mutate(body)
                elif path=='/api/prepare-next': result=app.prepare_next(body.get('episode_id'))
                elif path=='/api/review':
                    if type(body.get('keep')) is not bool: raise ValueError('keep must be boolean.')
                    result=app.review(body['episode_id'],body['keep'],body.get('success'))
                elif path=='/api/export':
                    ids=body.get('episode_ids',[])
                    if not isinstance(ids,list) or len(ids)>100 or not ids: raise ValueError('Select 1 to 100 episodes.')
                    if len(set(ids))!=len(ids): raise ValueError('Duplicate episode identifiers.')
                    result=app.export(ids)
                else: return self.reply({'error':'Not found'},404)
                return self.reply(result)
            except (ValueError,KeyError) as e: return self.reply({'error':str(e)},400)
            except Exception as e: return self.reply({'error':str(e)},500)

    server=ThreadingHTTPServer((config['host'],config['port']),Handler)
    server.daemon_threads=True
    (ROOT/'run.pid').write_text(str(os.getpid()))
    print(f'{config["mode"]} collector ready on port {config["port"]}; base disabled. PID={os.getpid()}',flush=True)

    def shutdown(signum,frame):
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,shutdown); signal.signal(signal.SIGINT,shutdown)
    try: server.serve_forever(poll_interval=.2)
    finally:
        reviews.close(); app.close(); server.server_close()
        (ROOT/'run.pid').unlink(missing_ok=True)
        lock.close()


if __name__=='__main__': main()
