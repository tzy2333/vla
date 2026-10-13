"""Start/stop/status for this collector only. No root privileges or other services."""
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parent
CONFIG=json.loads((ROOT/'config.json').read_text())
BASE=f'http://127.0.0.1:{CONFIG["port"]}'


def owned_pid():
    try:
        pid=int((ROOT/'run.pid').read_text())
        if pid<=1:return None
        proc=Path('/proc')/str(pid)
        args=(proc/'cmdline').read_bytes().split(b'\0')
        if (proc/'cwd').resolve()!=ROOT or not any(a in (b'run.py',str(ROOT/'run.py').encode()) for a in args):
            return None
        return pid
    except (OSError,ValueError):return None


def read_status():
    return json.load(urlopen(BASE+'/api/status',timeout=2))


def wait_ready(process=None):
    for i in range(60):
        if process is not None and process.poll() is not None:
            raise RuntimeError('启动退出，请检查 '+str(ROOT/'run.log'))
        try:
            s=read_status()
            if s.get('mode')=='real':
                print(f'采集服务已运行 PID={owned_pid()}；保存频率={s["fps"]}Hz；'
                      f'浏览器 http://192.168.1.34:{CONFIG["port"]}/')
                if not s['health']['robot']['healthy'] or not all(c['healthy'] for c in s['health']['cameras'].values()):
                    print('管理界面已在线；部分设备未就绪，录制保持禁用，请在设备页查看。')
                return
        except OSError:pass
        time.sleep(.25)
    raise RuntimeError('服务尚未就绪，请检查 '+str(ROOT/'run.log'))


def main():
    op=sys.argv[1] if len(sys.argv)>1 else 'status'
    if op not in ('start','stop','status'):raise SystemExit('用法：bash manage.sh start|status|stop')
    with (ROOT/'.management.lock').open('a+') as management_lock:
        fcntl.flock(management_lock,fcntl.LOCK_EX)
        pid=owned_pid()
        if op=='status':
            if not pid:print('采集服务未运行');return
            s=read_status()
            print(json.dumps({'pid':pid,'phase':s['phase'],'mode':s['mode'],'fps':s['fps'],
                'sample_interval_s':s.get('sample_interval_s'),'action_horizon':s.get('action_horizon'),
                'url':f'http://192.168.1.34:{CONFIG["port"]}/'},ensure_ascii=False))
        elif op=='start':
            if pid:wait_ready();return
            with socket.socket() as sock:
                if sock.connect_ex(('127.0.0.1',CONFIG['port']))==0:
                    raise RuntimeError('端口已被占用，未改动占用进程。')
            with (ROOT/'run.log').open('ab') as log:
                process=subprocess.Popen([str(ROOT/'.venv/bin/python'),'-u',str(ROOT/'run.py')],
                    cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
            wait_ready(process)
        else:
            if not pid:print('采集服务已停止');return
            s=read_status()
            if s['phase'] in ('RECORDING','EXPORTING'):
                raise RuntimeError('录制或导出正在运行，请先正常结束后停止服务。')
            os.kill(pid,signal.SIGTERM)
            for i in range(100):
                if owned_pid()!=pid:
                    print('采集服务已正常停止；原始文件保留。');return
                time.sleep(.2)
            raise RuntimeError('服务仍在完成落盘，未执行强制停止，请查看日志。')


if __name__=='__main__':
    try:main()
    except Exception as e:raise SystemExit(str(e))
