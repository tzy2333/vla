"""Revisioned human decisions and resumable automatic checks, outside raw data."""
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid
from collections import Counter
from .storage import atomic_json, load_json
from .platform_management import real_episode, now, text
from .quality_inspection import DEFAULTS, INSPECTION_VERSION, inspect_episode

BUSY=('PREPARING','RECORDING','EXPORTING')
ERROR_TYPES=('sensor','motion','annotation','format','sync','other')


class ReviewWorkflow:
    def __init__(self, app, start_worker=True):
        self.app=app; self.root=app.data/'reviews'; self.root.mkdir(exist_ok=True)
        self.lock=threading.RLock();self.stop=threading.Event();self.worker=None;self.active=None
        self.export_job=dict(status='idle'); self.export_thread=None
        if start_worker:
            self.worker=threading.Thread(target=self._loop,daemon=True);self.worker.start()

    def targets(self):
        p=self.root/'targets.json'
        return load_json(p) if p.exists() else dict(revision='0',values=DEFAULTS.copy())

    def save_targets(self,body):
        with self.lock:
            old=self.targets()
            if old['revision']!=body.get('revision'):raise ValueError('阈值已更新，请刷新。')
            values=body.get('values')
            if not isinstance(values,dict) or set(values)!=set(DEFAULTS):raise ValueError('阈值字段不完整。')
            for k,v in values.items():
                if type(v) not in (int,float) or not math.isfinite(v) or v<0 or v> (1 if k=='kappa' else 100000):raise ValueError('阈值无效：'+k)
                if k.endswith('_pct') and v>100:raise ValueError('百分比不能超过100。')
            if values['failure_min_pct']>values['failure_max_pct'] or not 1<=values['minimum_episodes']<=100000 or int(values['minimum_episodes'])!=values['minimum_episodes']:raise ValueError('样本范围无效。')
            doc=dict(revision=uuid.uuid4().hex,values=values,updated_at=now())
            atomic_json(self.root/'targets.json',doc);return doc

    def context(self,ident):
        raw=self.app.episode_path(ident);m=load_json(raw/'manifest.json')
        if not real_episode(m) or m.get('status')=='RECORDING':raise ValueError('只能审核已结束的真实录制。')
        return raw,m

    def load(self,ident):
        p=self.root/(ident+'.json')
        d=load_json(p) if p.exists() else dict(episode_id=ident,revision='0',state='pending_primary',cycle=0,
            primary=None,secondary=None,blind=None,outcome='unknown',history=[],l1=None,metadata={})
        # Compatibility is a view over existing decisions, never a new human vote.
        if d['state']=='pending_secondary' and (d.get('primary') or {}).get('decision')=='pass':
            d['state']='approved'
        if not any(h.get('stage') in ('primary','secondary','blind','arbitration','resubmit') for h in d['history']):
            m=load_json(self.app.episode_path(ident)/'manifest.json')
            if m.get('status') in ('COMMITTED','DISCARDED'):
                d['legacy_decision']='pass' if m['status']=='COMMITTED' else 'reject'
                d['state']='approved' if m['status']=='COMMITTED' else 'rejected'
                if m.get('success') is not None:d['outcome']='success' if m['success'] else 'failure'
        return d

    def read(self,ident):
        with self.lock:
            _,m=self.context(ident);d=self.load(ident)
            return dict(d,legacy_status=m.get('status'),raw_retained=True)

    def _save(self,d,human=True):
        if human:d['revision']=uuid.uuid4().hex
        d['updated_at']=now();atomic_json(self.root/(d['episode_id']+'.json'),d)
        return copy.deepcopy(d)

    def mutate(self,b):
        with self.lock,self.app.lock:
            if self.app.phase in BUSY:raise ValueError('请先结束录制或导出。')
            ident=b.get('episode_id');self.context(ident);d=self.load(ident)
            if b.get('revision')!=d['revision']:raise ValueError('另一条人工审核已提交，请重新打开核对后再提交。')
            reviewer=text(b.get('reviewer',''),80,'审核员',b.get('stage') not in ('primary','resubmit'))
            stage=b.get('stage');decision=b.get('decision');reason=text(b.get('reason',''),1000,'原因')
            outcome=b.get('outcome','unknown')
            if outcome not in ('unknown','success','failure','recovery'):raise ValueError('任务结果无效。')
            if stage=='resubmit':
                d.pop('legacy_decision',None)
                if d['state'] not in ('rejected','rework'):raise ValueError('仅返工或不通过片段可以重提。')
                if not reason:raise ValueError('请填写返工或重提说明。')
                d.update(state='pending_primary',cycle=d['cycle']+1,primary=None,secondary=None,blind=None)
            else:
                if stage not in ('primary','secondary','blind','arbitration') or decision not in ('pass','reject','rework'):raise ValueError('审核操作无效。')
                if decision!='pass' and not reason:raise ValueError('不通过/返工必须填写原因。')
                error=b.get('error_type','other')
                if error not in ERROR_TYPES:raise ValueError('错误类型无效。')
                if stage=='primary' and d['state']!='pending_primary':raise ValueError('当前不是待初审状态。')
                if stage=='secondary':
                    if d['state'] not in ('approved','pending_secondary') or d.get('secondary'):raise ValueError('仅已通过且尚未复审的片段可以复审。')
                    if reviewer==(d.get('primary') or {}).get('reviewer'):raise ValueError('复审必须由另一位审核员完成。')
                if stage=='blind':
                    if not d['primary'] or not d['primary'].get('reviewer') or reviewer==d['primary']['reviewer'] or d['blind']:raise ValueError('独立抽检须由另一位审核员完成，且同轮只提交一次。')
                    if b.get('independent') is not True:raise ValueError('请确认独立判定，未参考初审意见。')
                if stage=='arbitration':
                    if d['state'] not in ('rejected','rework','disputed') or not reason:raise ValueError('仲裁须针对争议/不通过/返工，并填写理由。')
                    if reviewer in {x['reviewer'] for x in (d.get('primary'),d.get('secondary'),d.get('blind')) if x}:raise ValueError('仲裁须由第三位审核员处理。')
                item=dict(stage=stage,decision=decision,reviewer=reviewer,reason=reason,error_type=error,
                          outcome=outcome,at=now(),cycle=d['cycle'])
                if stage in ('primary','secondary','blind'):d[stage]=item
                if stage=='blind':
                    if (decision=='pass')!=(d['primary']['decision']=='pass'):d['state']='disputed'
                else:
                    d['state']='approved' if decision=='pass' else ('rejected' if decision=='reject' else 'rework')
                    d['outcome']=outcome
            metadata=b.get('metadata',{})
            if not isinstance(metadata,dict) or set(metadata)-{'scene','object','lighting','environment','rubric'}:raise ValueError('元数据字段无效。')
            d['metadata'].update({k:text(v,500,k) for k,v in metadata.items()})
            d['history'].append(dict(stage=stage,decision=decision,reviewer=reviewer,reason=reason,
                                     error_type=b.get('error_type','other'),outcome=outcome,at=now(),cycle=d['cycle']))
            return self._save(d)

    def inspect(self,ident):
        raw,_=self.context(ident);targets=self.targets()
        def checkpoint():
            while self.app.phase in BUSY:
                if self.stop.wait(.2):raise InterruptedError()
            if self.stop.is_set():raise InterruptedError()
        report=inspect_episode(raw,targets['values'],checkpoint)
        report.update(checked_at=now(),targets_revision=targets['revision'])
        with self.lock:
            d=self.load(ident);d['l1']=report
            d['history'].append(dict(stage='automatic',at=now(),status=report['status'],targets_revision=targets['revision']))
            # Quality checks are advisory and do not invalidate human edits or approvals.
            return self._save(d,human=False)

    def queue_check(self,ident):
        with self.lock:
            self.context(ident);d=self.load(ident)
            if self.active==ident:raise ValueError('该片段正在检查。')
            if d['l1']:d['history'].append(dict(stage='automatic_previous',report=d['l1']))
            d['l1']=None
            return self._save(d,human=False)

    def _loop(self):
        while not self.stop.wait(3):
            try:
                if self.app.phase in BUSY:continue
                for ep in self.app.episodes():
                    if self.stop.is_set() or self.app.phase in BUSY:break
                    if not real_episode(ep) or (self.load(ep['episode_id'])['l1'] or {}).get('inspection_version')==INSPECTION_VERSION:continue
                    self.active=ep['episode_id']
                    try:self.inspect(self.active)
                    except InterruptedError:break
                    except Exception as exc:
                        with self.lock:
                            d=self.load(self.active);d['l1']=dict(status='error',error=str(exc),checked_at=now(),inspection_version=INSPECTION_VERSION);self._save(d,human=False)
                    finally:self.active=None
            except Exception:continue

    def snapshot(self,day='',task_id=''):
        with self.lock:
            entries=[];targets=self.targets()
            for ep in self.app.episodes():
                if not real_episode(ep):continue
                if day and not ep['episode_id'].startswith(day.replace('-','')):continue
                if task_id and ep.get('task_id')!=task_id:continue
                d=self.load(ep['episode_id']);entries.append(dict(episode=ep,review=d))
            ds=[e['review'] for e in entries];n=len(ds)
            count=lambda state:sum(d['state']==state for d in ds)
            legacy_pass=sum(d.get('legacy_decision')=='pass' for d in ds);legacy_reject=sum(d.get('legacy_decision')=='reject' for d in ds)
            primary=[d['primary'] for d in ds if d['primary']]; secondary=[d['secondary'] for d in ds if d['secondary']]
            rate=lambda a,b:round(100*a/b,2) if b else None
            pairs=[(d['primary']['decision']=='pass',d['blind']['decision']=='pass') for d in ds if d['primary'] and d['blind']]
            agreement=kappa=None
            if pairs:
                agreement=sum(a==b for a,b in pairs)/len(pairs)
                pa=sum(a for a,b in pairs)/len(pairs);pb=sum(b for a,b in pairs)/len(pairs);chance=pa*pb+(1-pa)*(1-pb)
                if chance<1:kappa=(agreement-chance)/(1-chance)
            l1=[d['l1'] for d in ds if d['l1'] and d['l1']['status'] in ('passed','failed')]
            stale=sum(bool(d['l1']) and d['l1'].get('targets_revision')!=targets['revision'] for d in ds)
            scores=[q['score'] for q in l1];critical=any(q.get('critical_failure') for q in l1)
            score=sum(scores)/len(scores) if scores else None
            outcomes=Counter(d['outcome'] for d in ds if d['outcome']!='unknown')
            errors=Counter(next((h.get('error_type','other') for h in reversed(d['history']) if h.get('reviewer')), 'other') for d in ds if d['state'] in ('rejected','rework','disputed'))
            diversity={k:dict(Counter(str((e['review']['metadata'].get(k) or (e['episode'].get('task_metadata') or {}).get(k) or (e['episode'].get('operator') if k=='operator' else '未填写'))) for e in entries)) for k in ('operator','scene','object','lighting')}
            delivered=count('approved')
            summary=dict(total=n,pending=count('pending_primary')+count('pending_secondary'),pending_primary=count('pending_primary'),pending_secondary=count('pending_secondary'),
                primary_passed=sum(x['decision']=='pass' for x in primary)+legacy_pass,primary_decided=len(primary)+legacy_pass+legacy_reject,
                primary_rate=rate(sum(x['decision']=='pass' for x in primary)+legacy_pass,len(primary)+legacy_pass+legacy_reject),
                secondary_passed=sum(x['decision']=='pass' for x in secondary),secondary_decided=len(secondary),
                secondary_rate=rate(sum(x['decision']=='pass' for x in secondary),len(secondary)),
                rejected=count('rejected'),rework=count('rework'),disputed=count('disputed'),final_delivery=delivered,
                rework_rate=rate(sum(any(x.get('decision')=='rework' or x.get('stage')=='resubmit' for x in d['history']) for d in ds),n),
                auto_checked=len(l1),auto_passed=sum(q['status']=='passed' for q in l1),auto_coverage=rate(len(l1),n),stale_checks=stale,
                task_success_rate=rate(outcomes['success'],outcomes['success']+outcomes['failure']),
                failure_recovery_pct=rate(outcomes['failure']+outcomes['recovery'],sum(outcomes.values())),
                outcomes=dict(outcomes),paired_count=len(pairs),agreement_pct=agreement*100 if agreement is not None else None,kappa=kappa,
                batch_score=score,batch_status='警告：存在数据异常' if critical else '待完整初筛' if len(l1)!=n or not n or stale else '警告：未达目标' if score<targets['values']['batch_score'] else '样本不足' if n<targets['values']['minimum_episodes'] else '已检测项目达标',
                error_types=dict(errors),diversity=diversity)
            return dict(entries=entries,summary=summary,targets=targets,active_check=self.active,export_job=copy.deepcopy(self.export_job))

    def start_export(self,b):
        ids=b.get('episode_ids');kind=b.get('kind','observations')
        if not isinstance(ids,list) or not 1<=len(ids)<=100 or len(set(ids))!=len(ids):raise ValueError('请选择1至100个不重复片段。')
        if kind!='observations':raise ValueError('动作目标尚未接入，不能导出训练数据；可导出真实观测 v2.1。')
        with self.lock,self.app.lock:
            if self.app.phase in BUSY or self.export_job['status']=='running':raise ValueError('请先结束当前采集或导出。')
            for ident in ids:
                self.context(ident);d=self.load(ident)
                if d['state']!='approved':raise ValueError('请先人工审核通过所选片段；无需等待自动检查或复审。')
            previous=self.app.phase;self.app.phase='EXPORTING'
            self.export_job=dict(status='running',started_at=now(),episode_ids=ids,format='v2.1')
            reviews={ident:copy.deepcopy(self.load(ident)) for ident in ids}
            def run():
                try:
                    from .observation_export import export_observations
                    result=export_observations([self.app.episode_path(i) for i in ids],self.app.exports,reviews,self.app.data/'annotations')
                    with self.lock:self.export_job.update(status='complete',result=result);self.app.last_export=result
                except Exception as exc:
                    with self.lock:self.export_job.update(status='failed',error=str(exc))
                finally:
                    with self.app.lock:self.app.phase=previous
            self.export_thread=threading.Thread(target=run,daemon=True);self.export_thread.start()
            return copy.deepcopy(self.export_job)

    def close(self):
        self.stop.set()
        if self.worker:self.worker.join(timeout=5)
