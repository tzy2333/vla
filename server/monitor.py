#!/usr/bin/env python3
"""Standalone, read-only live terminal. Never connects to hardware or sends POSTs."""
import argparse
from collections import deque
import datetime
import json
import math
import shutil
import sys
import time
import unicodedata
from urllib.request import build_opener, ProxyHandler


def compact(text, width):
    result='';used=0
    for c in str(text).replace('\n',' ').replace('\r',' '):
        if unicodedata.category(c).startswith('C'):
            continue
        size=2 if unicodedata.east_asian_width(c) in ('W','F') else 1
        if used+size>width:break
        result+=c;used+=size
    return result


def padded(text,width):
    text=compact(text,width)
    return text+' '*max(0,width-sum(2 if unicodedata.east_asian_width(c) in ('W','F') else 1 for c in text))


def source_label(key,status):
    cameras=status.get('capabilities',{}).get('cameras',[])
    label=next((c.get('label',key) for c in cameras if c['id']==key),None)
    return (label+'相机') if label else {'head':'头部相机','left_wrist':'左腕相机','right_wrist':'右腕相机','robot':'机器人反馈'}.get(key,key)


def read_channel(packet,channel):
    value=packet
    try:
        for key in channel.get('packet_path',[]):value=value[key]
    except (KeyError,IndexError,TypeError):return None
    return value if type(value) in (int,float) and math.isfinite(value) else None


class Feed:
    def __init__(self):
        self.seen={};self.phase=None;self.written=None

    def events(self, status):
        out=[]
        packet=status.get('robot_feedback')
        desc=status.get('capabilities',{}).get('robot',{})
        recording=status.get('phase')=='RECORDING'
        chosen=status.get('capture_selection') if recording else None
        if packet and status.get('feedback_valid'):
            ident=(packet.get('session'),packet.get('packet_id'),packet.get('received_ns'))
            if ident!=self.seen.get('robot'):
                self.seen['robot']=ident
                for module in desc.get('modules',[]):
                    if chosen and module['id'] not in chosen['modules']:continue
                    channels=[c for c in desc['channels'] if c['id'] in module['channels']]
                    values=[]
                    for c in channels:
                        v=packet['state'][c['index']]
                        values.append(f'{math.degrees(v):+.2f}deg' if c['unit']=='rad' else f'{v:+.4f}{c["unit"]}')
                    out.append(('JOINT',module.get('label',module['id']),f'收到关节数据  #{packet["packet_id"]:08d}  ['+', '.join(values)+']'))
                for effector in desc.get('end_effectors',[]):
                    values=[read_channel(packet,c) for c in effector.get('channels',[])]
                    if values and all(v is not None for v in values):
                        out.append(('HAND',effector['label'],'收到原始反馈 · 实时监测  ['+', '.join(f'{v:.3f}' for v in values)+']'))
        for key,h in status.get('health',{}).get('cameras',{}).items():
            if chosen and key not in chosen['cameras']:continue
            ident=(h.get('camera_session'),h.get('frame_id'),h.get('received_ns'))
            if h.get('healthy') and h.get('frame_id') is not None and ident!=self.seen.get(key):
                self.seen[key]=ident
                age=f'{h["age_ms"]:.1f} ms' if isinstance(h.get('age_ms'),(int,float)) else '—'
                out.append(('IMAGE',source_label(key,status),f'收到图像  #{h["frame_id"]:08d}  {h["width"]}×{h["height"]}  帧龄 {age}  图像正常'))
        phase=status.get('phase')
        if phase!=self.phase:
            self.phase=phase
            out.append(('STATUS','采集服务',{'RECORDING':'正在采集并写入数据','IDLE':'实时监测 · 未录制','REVIEW':'录制结束 · 待审核','FAULT':'录制故障'}.get(phase,phase)))
        written=(status.get('episode_id'),status.get('written_frames'))
        if phase=='RECORDING' and written!=self.written:
            self.written=written
            out.append(('WRITE','数据存储',f'已保存 {status.get("written_frames",0):,} 组样本  |  待写入 {status.get("writer_queue",0)}  |  {status.get("episode_id")}'))
        for key,h in [('robot',status.get('health',{}).get('robot',{})),*status.get('health',{}).get('cameras',{}).items()]:
            state=(h.get('healthy'),h.get('error'))
            if state!=self.seen.get('health:'+key):
                previous=self.seen.get('health:'+key);self.seen['health:'+key]=state
                if not h.get('healthy'):out.append(('WARN',source_label(key,status),h.get('error') or '等待数据'))
                elif previous:out.append(('STATUS',source_label(key,status),'数据链路恢复'))
        return out


class Terminal:
    def __init__(self, plain=False):
        self.ansi=sys.stdout.isatty() and not plain
        self.size=None

    def header(self,status,rate,error=None):
        size=shutil.get_terminal_size((120,32));self.width=max(20,size.columns-1)
        phase=status.get('phase','OFFLINE');stats=status.get('recording_pull_statistics') or {}
        success=stats.get('success_rate');hz=stats.get('effective_hz')
        robot=status.get('capabilities',{}).get('robot',{}).get('label','ROBOT DATA ACQUISITION')
        phases={'RECORDING':'正在采集 · 数据写入中','IDLE':'实时监测 · 未录制','REVIEW':'实时监测 · 片段待审核','FAULT':'录制中断 · 检查设备','OFFLINE':'服务离线'}
        title=f' 机器人数据采集  /  {robot}  |  '+phases.get(phase,str(phase))
        scope='本次录制' if phase=='RECORDING' else '最近录制'
        detail=f' {scope}成功率 {success:.1f}%' if success is not None else ' 关节成功率 —'
        detail+=f'  |  有效响应 {hz:.1f} Hz' if hz is not None else '  |  有效响应 —'
        detail+=f'  |  展示上限 {rate:g} 行/秒  |  Ctrl+C 退出'
        if error:title=' 连接中断 · 等待恢复';detail=' '+str(error)
        if self.ansi:
            if self.size!=size:
                self.size=size
                sys.stdout.write(f'\033[0m\033[2J\033[4;{max(5,size.lines)}r\033[4;1H\033[?25l')
            sys.stdout.write('\0337\033[1;1H\033[2K\033[1;36m'+compact(title,self.width)+
                             '\033[0m\033[2;1H\033[2K'+compact(detail,self.width)+
                             '\033[3;1H\033[2K\033[2m'+'─'*self.width+'\033[0m\0338')
            sys.stdout.flush()

    def line(self,event):
        kind,source,body=event
        stamp=datetime.datetime.now().astimezone().strftime('%H:%M:%S.%f')[:-3]
        label={'JOINT':'关节反馈','HAND':'手部反馈','IMAGE':'图像接收','WRITE':'数据落盘','STATUS':'链路状态','WARN':'设备状态'}.get(kind,kind)
        text=compact(f'{stamp}  {padded(label,8)}  {padded(source,14)}  {body}',getattr(self,'width',160))
        if self.ansi:
            color='33' if kind=='WARN' else '92' if kind=='WRITE' else '32' if kind=='JOINT' else '35' if kind=='HAND' else '36'
            print(f'\033[{color}m{text}\033[0m',flush=True)
        else:print(text,flush=True)

    def close(self):
        if self.ansi:print('\033[r\033[0m\033[?25h',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',default='http://127.0.0.1:8766')
    p.add_argument('--rate',type=float,default=40,help='Maximum displayed lines per second, 1..60')
    p.add_argument('--poll',type=float,default=15,help='Read-only API polls per second, 1..20')
    p.add_argument('--duration',type=float,default=0,help='Optional bounded run in seconds')
    p.add_argument('--plain',action='store_true',help='Plain scroll without fixed header or ANSI')
    args=p.parse_args()
    if not 1<=args.rate<=60 or not 1<=args.poll<=20 or not math.isfinite(args.duration) or args.duration<0:
        p.error('Invalid rate, poll frequency, or duration.')
    feed=Feed();terminal=Terminal(args.plain);opener=build_opener(ProxyHandler({}))
    queue=deque(maxlen=max(4,round(args.rate*.5)));start=time.monotonic();next_poll=start;next_line=start
    status={};last_error=None
    try:
        while not args.duration or time.monotonic()-start<args.duration:
            now=time.monotonic()
            if now>=next_poll:
                try:
                    with opener.open(args.url.rstrip('/')+'/api/status',timeout=2) as response:status=json.load(response)
                    if last_error:queue.clear();feed=Feed();queue.append(('STATUS','SERVER','连接恢复'))
                    last_error=None;queue.extend(feed.events(status));terminal.header(status,args.rate)
                    next_poll=time.monotonic()+1/args.poll
                except Exception as exc:
                    error=f'{type(exc).__name__}: {exc}'
                    queue.clear()
                    if error!=last_error:queue.append(('WARN','SERVER',error))
                    last_error=error;terminal.header({},args.rate,error);next_poll=time.monotonic()+1
            now=time.monotonic()
            if queue and now>=next_line:
                terminal.line(queue.popleft());next_line=now+1/args.rate
            time.sleep(.005)
    except KeyboardInterrupt:pass
    finally:terminal.close()


if __name__=='__main__':main()
