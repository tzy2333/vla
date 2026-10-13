// Capability-driven acquisition UI. Selection controls recording, not just visibility.
let startingCapture=false;
let replayPlaying=false,replayLoading=false,replayTimer=null,replayRun=0,replayFps=0,replayIndex=0;
let captureChoice=null,capabilitySignature='',cameraSignature='',jointSignature='',activeCameraOperation=null;
const cameraLabel=k=>status.capabilities?.cameras.find(c=>c.id===k)?.label||({head:'头部',left_wrist:'左腕',right_wrist:'右腕'}[k]||k);
function selectedChoice(){return captureChoice||status.capabilities?.default_selection||{modules:[],cameras:[]};}
function syncCapabilities(){
  const cap=status.capabilities;if(!cap)return;
  const signature=JSON.stringify(cap);
  if(signature!==capabilitySignature){
    capabilitySignature=signature;captureChoice=captureChoice?{modules:captureChoice.modules.filter(k=>cap.robot.modules.some(m=>m.id===k)),cameras:captureChoice.cameras.filter(k=>cap.cameras.some(c=>c.id===k))}:structuredClone(cap.default_selection);
    $('robotModel').textContent=cap.robot.label;
    for(const [rootId,kind,items] of [['moduleChoices','modules',cap.robot.modules],['cameraChoices','cameras',cap.cameras]]){
      const root=$(rootId);root.replaceChildren();
      for(const item of items){const label=node('label','module-chip'),input=node('input');input.type='checkbox';input.value=item.id;input.dataset.selection=kind;input.checked=captureChoice[kind].includes(item.id);label.append(input,node('span','',item.label));
        if(kind==='cameras'){const row=node('div','capture-camera-option'),head=node('div','capture-camera-header'),badge=node('span','badge');row.dataset.captureCamera=item.id;badge.dataset.cameraStatus=item.id;label.append(node('small','muted','录制'));head.append(label,badge);row.append(head,cameraAction(item.id,status.health?.cameras?.[item.id]));root.append(row);}else root.append(label);
        input.onchange=()=>{captureChoice[kind]=items.filter(i=>root.querySelector('input[value="'+i.id+'"]').checked).map(i=>i.id);syncCameraLayout();controls();preview();};}
    }
    $('moduleNote').textContent='';
  }
  if(status.phase==='RECORDING'&&status.capture_selection)captureChoice=structuredClone(status.capture_selection);
  for(const input of document.querySelectorAll('[data-selection]'))input.checked=captureChoice[input.dataset.selection].includes(input.value);
  if(!replaying)syncCameraLayout();
}
function syncCameraLayout(keys=null){
  const wanted=keys||(replaying?Object.keys(replayRow?.images||{}):selectedChoice().cameras);
  const ordered=[...wanted].sort((a,b)=>{const order=['head','left_wrist','right_wrist'];const rank=k=>order.includes(k)?order.indexOf(k):99;return rank(a)-rank(b);});
  cameras=ordered;
  const signature=JSON.stringify(ordered);if(signature===cameraSignature)return;cameraSignature=signature;
  const grid=document.querySelector('.cameras');grid.replaceChildren();grid.dataset.count=String(ordered.length);grid.classList.toggle('head-top',ordered.length===3&&ordered.includes('head'));
  for(const key of ordered){const card=node('div','camera');card.dataset.camera=key;const label=node('label','',cameraLabel(key)),meta=node('span','muted');label.append(meta);const img=node('img');img.id=key;img.alt=cameraLabel(key)+'相机画面';const fallback=node('div','camera-fallback','等待画面');fallback.id=key+'Fallback';
    img.onload=()=>{fallback.hidden=false;const h=status.health?.cameras?.[key];if(replaying||h?.healthy)fallback.hidden=true;};
    img.onerror=()=>{fallback.hidden=false;fallback.textContent=replaying?'回放图像不可用':'等待图像';};card.append(label,img,fallback);grid.append(card);}
  if(!ordered.length)grid.append(node('div','camera-empty',replaying?'此片段未录制图像':'未选择相机'));
}
function updateDevices(){
  syncCapabilities();const h=status.health;if(!h)return;
  const strip=$('cameraPills');strip.replaceChildren();
  for(const [key,c] of Object.entries(h.cameras)){
    const released=c.error==='相机已释放';
    const pill=node('div','device-pill'+(c.healthy||released?'':' bad'));pill.append(node('span','dot'),node('span','',cameraLabel(key)+' · '+(released?'已释放':c.healthy?'在线':'离线')));strip.append(pill);
    const img=$(key);if(img&&!replaying){img.parentElement.querySelector('label .muted').textContent=c.width+'×'+c.height+' · '+c.native_fps+' fps';if(!c.healthy){$(key+'Fallback').hidden=false;$(key+'Fallback').textContent=released?'相机已释放':'图像超时';}}
  }
  $('robotPill').className='device-pill'+(h.robot.healthy?'':' bad');$('robotPill').lastElementChild.textContent='关节 · '+(h.robot.healthy?'在线':'离线');
  $('diskSpace').textContent='可用 '+(h.disk_free_mb/1024).toFixed(1)+' GB';
  $('storageAlert').hidden=h.disk_free_mb>=5120;$('storageAlert').textContent='存储空间剩余 '+(h.disk_free_mb/1024).toFixed(1)+' GB';
  const root=$('deviceChecks'),devices=[['robot',h.robot],...Object.entries(h.cameras)],signature=JSON.stringify(devices.map(([key])=>key));
  if(root.dataset.signature!==signature){root.dataset.signature=signature;root.replaceChildren();
    for(const [key] of devices){const card=node('div','device-check');card.dataset.deviceKey=key;card.append(node('strong','device-name'),node('span','badge device-state'),node('span','muted device-serial'),node('span','muted device-error'));if(key!=='robot')card.append(cameraAction(key,h.cameras[key]));root.append(card);}}
  for(const [key,d] of devices){const card=[...root.children].find(c=>c.dataset.deviceKey===key),released=d.error==='相机已释放';card.querySelector('.device-name').textContent=key==='robot'?(status.capabilities?.robot.label||'机器人'):cameraLabel(key);const badge=card.querySelector('.device-state');badge.textContent=released?'已释放':d.healthy?'在线':'离线';badge.className='badge device-state '+(d.healthy?'green':released?'':'red');card.querySelector('.device-serial').textContent=d.serial?'序列号 '+d.serial:'';card.querySelector('.device-error').textContent=d.error||'';}
  $('sourceMessage').textContent=status.capabilities?.robot.notes||'状态来自当前设备接口。';$('dataPath').textContent=status.data_path;
  renderLiveJoints();renderHands();renderPullStats();updateCameraActions();
}
function renderLiveJoints(){
  const legacy={channels:[],modules:[]};
  for(const [side,offset,label] of [['left',0,'左臂'],['right',6,'右臂']]){const ids=[];['肩前抬','肩外摆','大臂旋转','肘关节','腕旋转'].forEach((name,j)=>{const id=side+'_legacy_'+j;ids.push(id);legacy.channels.push({id,label:name,index:offset+j,unit:'rad'});});legacy.modules.push({id:side+'_arm',label,channels:ids});}
  const desc=replaying?(episodes.find(e=>e.episode_id===replaying)?.robot_descriptor||legacy):status.capabilities?.robot;
  const channels=replaying?replayRow?.state_channels:status.state_channels;
  const state=replaying?replayRow?.['observation.state']:status.state;
  const available=channels||desc?.channels||[];
  const signature=JSON.stringify([available.map(c=>c.id),replaying]);
  if(signature!==jointSignature){jointSignature=signature;const root=$('liveJoints');root.replaceChildren();
    for(const group of desc?.modules||[]){const filtered=available.filter(c=>group.channels.includes(c.id));if(!filtered.length)continue;const panel=node('div','joint-group');panel.append(node('h3','',group.label));
      for(const c of filtered){const row=node('div','joint-line'),number=node('strong','joint-number');number.append(node('span','numeric-value','—'),node('small','numeric-unit',c.unit==='rad'?'°':c.unit));number.dataset.channel=c.id;row.append(node('span','',c.label),number);panel.append(row);}root.append(panel);}
    if(!available.length)root.append(node('p','muted',replaying?'此片段未录制关节':'等待设备描述'));}
  available.forEach((c,i)=>{const el=document.querySelector('[data-channel="'+c.id+'"]');if(!el)return;const v=state?.[replaying&&!channels?c.index:i],display=c.unit==='rad'?v*180/Math.PI:v,digits=c.unit==='rad'?2:4;el.querySelector('.numeric-value').textContent=Number.isFinite(v)?(Math.abs(display)<.5*10**(-digits)?0:display).toFixed(digits):'—';});
  $('jointStatus').textContent=replaying?'片段回放':status.feedback_valid?'实时反馈':'等待反馈';$('jointStatus').className='badge '+(replaying?'':status.feedback_valid?'green':'amber');
  $('jointAge').textContent=!replaying&&Number.isFinite(status.health?.robot?.age_ms)?'更新 '+status.health.robot.age_ms.toFixed(0)+' ms 前':'';
}
function renderPullStats(){
  const ep=episodes.find(e=>e.episode_id===replaying),stats=replaying?ep?.pull_statistics:status.recording_pull_statistics;
  $('pullRate').textContent=Number.isFinite(stats?.success_rate)?stats.success_rate.toFixed(1)+'%':'—';
  $('pullHz').textContent=Number.isFinite(stats?.effective_hz)?stats.effective_hz.toFixed(1)+' Hz':'—';
  $('pullDetail').textContent=stats?stats.successes+' / '+stats.attempts+' · 超时 '+stats.timeouts+' · 错误 '+stats.errors+' · 待回包 '+stats.pending:'开始录制后统计';
  $('pullScope').textContent=replaying?'本段统计':status.phase==='RECORDING'?'本次录制':'最近录制';
}
function renderMetrics(){const ep=episodes.find(e=>e.episode_id===replaying),count=ep?ep.frame_count:status.written_frames||0,rate=ep?ep.fps:status.fps;$('duration').textContent=(ep?count/rate:status.elapsed_seconds||0).toFixed(1)+' s';$('frameCount').textContent=count;$('sampleHz').textContent=(rate||50)+' Hz';$('duration').previousElementSibling.textContent=ep?'本轮时长':'本轮已录时长';$('frameCount').previousElementSibling.textContent=ep?'本轮样本':'已写入样本';}
function renderWorkMode(){
  const replay=!!replaying,p=status.phase,ep=reviewEpisode();
  const kind=replay?'replay':p==='RECORDING'?'recording':p==='PREPARING'||startingCapture?'preparing':'capture';
  $('workModeBar').dataset.mode=kind;document.body.dataset.workMode=kind;
  $('workModeTitle').textContent=replay?'历史回放 · '+(reviewNames[ep?.status]||'查看片段'):kind==='recording'?'正在录制':kind==='preparing'?'正在准备录制':'实时采集';
  $('workModeHint').textContent=replay?'当前查看已保存片段。审核可稍后完成，返回采集即可准备下一轮。':kind==='recording'?'正在保存本轮数据，结束后进入回放。':kind==='preparing'?(status.preparation?.message||'正在连接所选相机并等待新图像，请稍候。'):p==='REVIEW'?'上一轮已落盘、待审核；可直接开始下一轮，历史片段继续保留。':'开始时自动连接所选相机，无需提前逐台启用。';
  $('returnToCapture').hidden=!replay;$('returnToCapture').disabled=busy;
  $('captureSetup').hidden=replay;$('capturePanelTitle').textContent=replay?'回放与审核':'本轮录制';
  $('reviewDecision').hidden=!replay;$('returnCaptureSide').hidden=!replay;$('returnCaptureSide').disabled=busy;
  $('reviewCurrent').hidden=replay;
  $('reviewDecisionStatus').textContent=!ep?'':ep.status==='REVIEW'?'尚未审核不影响下一轮录制。':ep.status==='COMMITTED'?'此片段已保留，可返回采集继续录制。':ep.status==='DISCARDED'?'此片段已标记舍弃，原始文件仍保留。':reviewNames[ep.status]||ep.status;
}
function controls(){
  const p=status.phase,h=status.health,choice=selectedChoice(),locked=busy||!connected,idle=['IDLE','REVIEW'].includes(p),writing=(status.writer_queue||0)>0;
  const selectedAny=choice.modules.length+choice.cameras.length>0,robotReady=!choice.modules.length||h?.robot?.healthy,cameraReady=choice.cameras.every(k=>h?.cameras[k]?.healthy),rate=Number($('sampleRate').value),rateOK=choice.cameras.every(k=>(h?.cameras[k]?.native_fps||0)>=rate),diskReady=(h?.disk_free_mb||0)>=(status.minimum_free_mb||512);
  for(const id of ['task','operator','sampleRate','capturePurpose'])$(id).disabled=locked||!idle||writing||!!replaying;
  for(const input of document.querySelectorAll('[data-selection]'))input.disabled=locked||!idle||writing||!!replaying;
  $('capturePurpose').querySelector('option[value=training]').disabled=true;
  const blockers=[];
  if(!connected)blockers.push('服务连接中断，请重新连接');
  if(p==='PREPARING'||startingCapture)blockers.push(status.preparation?.message||'正在连接相机并等待新图像');
  else if(busy)blockers.push('正在处理操作，请稍候');
  if(replaying)blockers.push('当前为历史回放，请点击“返回采集，录制下一轮”');
  if(writing)blockers.push('上一轮正在落盘，请稍候');
  if(p==='FAULT')blockers.push('上一轮故障，请查看故障信息并处理该片段');
  if(p==='EXPORTING')blockers.push('正在导出，请稍候');
  if(!status.capabilities)blockers.push('设备配置尚未加载');
  if(!selectedAny)blockers.push('请选择要录制的模块或相机');
  if(!robotReady)blockers.push('所选关节反馈未就绪，请在本体与设置中重连关节反馈');
  if(!rateOK)blockers.push('采集频率高于相机帧率，请降低采集频率');
  if(!diskReady)blockers.push('存储空间不足');
  if(!$('task').value.trim())blockers.push('请填写任务描述');
  $('purposeHint').textContent='一次开始 / 结束计一轮；相机将在开始时自动启用。';
  $('start').textContent=startingCapture||p==='PREPARING'?'正在准备设备…':p==='REVIEW'?'开始下一轮':'开始本轮';
  $('start').hidden=p==='RECORDING';$('start').disabled=locked||!idle||writing||!!replaying||!selectedAny||!robotReady||!rateOK||!diskReady||!$('task').value.trim()||!status.capabilities;
  $('stop').hidden=p!=='RECORDING';$('stop').disabled=locked||p!=='RECORDING';
  const ep=reviewEpisode(),reviewable=['IDLE','REVIEW','FAULT'].includes(p);$('reviewBox').hidden=!ep;$('reviewLabel').textContent=ep?ep.task+' · '+ep.frame_count+' 组':'';
  $('reviewCurrent').disabled=locked||!reviewable||!ep?.frame_count;
  $('keep').disabled=locked||!reviewable||ep?.status!=='REVIEW'||replaying!==ep?.episode_id||!replayRow;
  $('discard').disabled=locked||!reviewable||!['REVIEW','FAULT','RECOVERED'].includes(ep?.status)||!replaying;
  $('success').disabled=locked||!reviewable||ep?.status!=='REVIEW';$('export').disabled=true;
  $('hint').textContent=p==='RECORDING'?'正在保存本轮数据':blockers.length?blockers.join('；'):status.preparation_error?'上次未能开始：'+status.preparation_error+' 可重新点击开始。':!cameraReady?'点击开始后将自动启用所选相机，就绪后录制。':p==='REVIEW'?'可开始下一轮，上一轮仍保留待审核。':'设备就绪，可以开始录制。';
  $('hint').dataset.blocked=String(blockers.length>0&&p!=='RECORDING');
  $('reconnectFeedback').disabled=locked||['PREPARING','RECORDING','EXPORTING'].includes(p)||writing;
  $('releaseCameras').disabled=$('reconnectFeedback').disabled;
  renderSelection();if(typeof managementControls==='function')managementControls();
  if(status.capabilities?.robot.id!=='rf_udp_20010')$('resetRobot').disabled=true;
  for(const id of ['robotAdapter','saveInterface'])$(id).disabled=locked||['PREPARING','RECORDING','EXPORTING'].includes(p)||!settingsData;
  if(typeof segmentControls==='function')segmentControls();
  renderWorkMode();
  if(typeof centralReviewControls==='function')centralReviewControls();
}
function values(){renderLiveJoints();renderHands();const state=replaying?replayRow?.['observation.state']:status.state;const channels=replaying?replayRow?.state_channels:status.state_channels;for(const [side,id] of [['left','leftState'],['right','rightState']]){$(id).textContent=(channels||[]).map((c,i)=>({c,v:state?.[i]})).filter(x=>x.c.id.startsWith(side+'_')).map(x=>x.c.label+' '+(Number.isFinite(x.v)?x.v.toFixed(4)+' '+x.c.unit:'—')).join('\n')||'—';}}
function renderTiming(){const q=replaying?replayRow?.quality:status.metrics;$('qualityTitle').textContent=replaying?'回放 · 时间对齐':'时间对齐';$('pairSkew').textContent=fmt(q?.camera_pair_skew_ms);$('imageOffset').textContent=fmt(replaying?Math.max(...Object.values(replayRow?.images||{}).map(i=>Math.abs(i.alignment_offset_ms))):q?.camera_alignment_offset_ms);$('interpolationGap').textContent=fmt(replaying?replayRow?.alignment?.joints?.gap_ms:q?.interpolation_gap_ms);$('timingStatus').textContent='主机时间对齐；曝光同步尚未标定。';}
function replayClock(seconds){if(!Number.isFinite(seconds))return '—';return Math.floor(seconds/60)+':'+String(Math.floor(seconds%60)).padStart(2,'0');}
function updateReplayControls(){
  const max=Number($('replaySlider').max),validRate=Number.isFinite(replayFps)&&replayFps>0;
  $('replayPlay').textContent=replayPlaying?'暂停':replayIndex>=max&&max>0?'重新播放':'播放';
  $('replayPlay').disabled=!replaying||!replayRow||!validRate||max<=0;
  $('replayRestart').disabled=!replaying||!replayRow||!validRate||max<=0;
  $('replayTime').textContent=replayClock(validRate?replayIndex/replayFps:NaN)+' / '+replayClock(validRate?(max+1)/replayFps:NaN);
  $('replayStatus').textContent=!validRate?'缺少录制帧率，可逐帧查看':replayPlaying?(replayLoading?'正在缓冲':'正在播放'):replayLoading?'正在加载':replayIndex>=max&&max>0?'播放结束':'已暂停';
}
function pauseReplay(){
  replayPlaying=false;++replayRun;++replayRevision;clearTimeout(replayTimer);replayTimer=null;replayLoading=false;
  if(replayRow)$('replaySlider').value=replayIndex;
  updateReplayControls();
  if(typeof segmentControls==='function')segmentControls();
}
async function playReplay(restart=false){
  if(!replaying||!replayRow||!(replayFps>0)||!Number.isFinite(replayFps))return;
  pauseReplay();const run=replayRun,ep=replaying,max=Number($('replaySlider').max);
  if(max<=0)return;
  if(restart||replayIndex>=max){if(!await replayFrame(0)||run!==replayRun||ep!==replaying)return;}
  replayPlaying=true;updateReplayControls();segmentControls();
  const anchorFrame=replayIndex,anchorTime=performance.now();
  async function tick(){
    if(!replayPlaying||run!==replayRun||ep!==replaying)return;
    const target=Math.min(max,anchorFrame+Math.floor((performance.now()-anchorTime)*replayFps/1000));
    if(target>replayIndex&&!await replayFrame(target))return;
    if(run!==replayRun||ep!==replaying||!replayPlaying)return;
    if(replayIndex>=max){pauseReplay();return;}
    replayTimer=setTimeout(tick,40);
  }
  replayTimer=setTimeout(tick,0);
}
function loadReplayImage(url){
  return new Promise((resolve,reject)=>{const img=new Image(),timer=setTimeout(()=>finish(Error('回放图像加载超时')),8000);
    function finish(error){clearTimeout(timer);img.onload=null;img.onerror=null;error?reject(error):resolve(img);}
    img.onload=()=>finish();img.onerror=()=>finish(Error('回放图像读取失败'));img.src=url;
  });
}
async function replayFrame(index){
  if(!replaying)return false;
  const revision=++replayRevision,ep=replaying;replayLoading=true;updateReplayControls();segmentControls();
  try{
    const row=await api('/api/replay?episode='+encodeURIComponent(ep)+'&frame='+index);
    if(revision!==replayRevision||replaying!==ep)return false;
    const images=await Promise.all(Object.keys(row.images||{}).map(async key=>[key,await loadReplayImage('/api/frame?episode='+encodeURIComponent(ep)+'&frame='+index+'&camera='+encodeURIComponent(key))]));
    if(revision!==replayRevision||replaying!==ep)return false;
    // Commit the same saved frame across images, slider and joint readout together.
    replayRow=row;replayIndex=index;syncCameraLayout(Object.keys(row.images||{}));values();renderPullStats();
    $('replaySlider').value=index;$('replayFrame').textContent=(index+1)+' / '+(Number($('replaySlider').max)+1);
    $('previousFrame').disabled=index<=0;$('nextFrame').disabled=index>=Number($('replaySlider').max);
    for(const [key,img] of images){$(key).src=img.src;$(key+'Fallback').hidden=true;}
    renderTiming();controls();return true;
  }catch(e){if(revision===replayRevision&&replaying===ep){pauseReplay();toast(e.message);}return false;}
  finally{if(revision===replayRevision){replayLoading=false;updateReplayControls();segmentControls();}}
}
$('replayPlay').onclick=()=>replayPlaying?pauseReplay():void playReplay();
$('replayRestart').onclick=()=>void playReplay(true);
$('replaySlider').oninput=()=>{const index=Number($('replaySlider').value);pauseReplay();$('replaySlider').value=index;void replayFrame(index);};
for(const [id,delta] of [['previousFrame',-1],['nextFrame',1]])$(id).onclick=()=>{pauseReplay();void replayFrame(Math.max(0,Math.min(Number($('replaySlider').max),replayIndex+delta)));};
document.addEventListener('visibilitychange',()=>{if(document.hidden)pauseReplay();});
function live(){pauseReplay();replaying=null;replayRow=null;++replayRevision;$('live').hidden=true;$('replayControls').hidden=true;$('previewMode').textContent='实时画面';$('previewNote').textContent='预览频率独立于录制。';$('stateMode').textContent='实时反馈';syncCameraLayout();renderTiming();renderMetrics();values();renderPullStats();controls();preview();}
function preview(){if(!connected||replaying||document.hidden)return;for(const key of cameras){const img=$(key);if(!img?.complete)continue;if(!status.health?.cameras[key]?.healthy){$(key+'Fallback').hidden=false;continue;}img.src='/preview/'+encodeURIComponent(key)+'.jpg?t='+Date.now();}}
$('start').onclick=()=>{if(busy||replaying)return;startingCapture=true;void action(async()=>{try{if(!token)token=(await api('/api/session')).token;const result=await api('/api/start',{task:$('task').value,task_id:$('captureTask').value||null,operator:$('operator').value,fps:Number($('sampleRate').value),purpose:'observation',capture_selection:structuredClone(selectedChoice())});status.phase=result.phase;$('success').value='unknown';toast('相机已就绪，录制已开始');}finally{startingCapture=false;}});};
$('returnToCapture').onclick=$('returnCaptureSide').onclick=()=>{live();$('start').scrollIntoView({block:'center',behavior:'smooth'});};
$('reconnectFeedback').onclick=()=>action(async()=>{const result=await api('/api/robot/reconnect',{});toast(result.message);});
$('releaseCameras').onclick=()=>action(async()=>{const result=await api('/api/cameras/release',{});toast(result.message);});

function renderSettings(data){
  settingsData=data;
  const select=$('robotAdapter');select.replaceChildren();
  for(const a of data.adapters||[])select.add(new Option(a.label,a.id));
  select.value=data.saved_adapter||'rf_udp_20010';renderInterfaceFields();
  $('robotName').value=data.identity.name;$('robotLocation').value=data.identity.location||'';
  $('settingsFps').value=String(data.saved.fps);
  if(![...$('cameraProfile').options].some(o=>o.value==='custom'))$('cameraProfile').add(new Option('按相机配置','custom'));
  $('cameraProfile').value='custom';
  const pending=data.camera_pending||data.model_pending;
  $('settingsPending').textContent=pending?'待应用':'已应用';$('settingsPending').className='badge '+(pending?'amber':'green');
  const name=data.adapters.find(a=>a.id===data.current_adapter)?.label||data.current_adapter;
  const bindings=data.current_camera_bindings||{};
  $('activeConfiguration').textContent=name+' · '+data.current.fps+' Hz\n'+Object.entries(bindings).map(([k,c])=>(c.label||cameraLabel(k))+' '+c.width+'×'+c.height+' · '+c.fps+' fps').join('\n');
  $('rfModelPanel').hidden=!data.model_supported;$('resetRobot').hidden=!data.model_supported;$('robotHistoryPanel').hidden=!data.model_supported;
  if(data.model_supported)renderRFModel(data);
  $('cameraSettings').replaceChildren();
  for(const [key,binding] of Object.entries(data.camera_bindings||{}))addCameraRow(key,binding);
  renderSettingsFeedback();renderMotorParameters();renderEndEffectorSettings();managementControls();
}
function renderInterfaceFields(){
  const adapter=settingsData?.adapters.find(a=>a.id===$('robotAdapter').value),root=$('interfaceFields');root.replaceChildren();
  for(const field of adapter?.fields||[]){
    const label=node('label','field',field.label||field.key),input=node('input');input.dataset.interfaceField=field.key;
    input.type=field.type==='boolean'?'checkbox':field.type==='text'?'text':'number';
    if(input.type==='checkbox')input.checked=adapter.values?.[field.key]??field.default??false;
    else input.value=adapter.values?.[field.key]??field.default??'';
    if(field.min!==undefined)input.min=field.min;if(field.max!==undefined)input.max=field.max;
    if(input.type==='number')input.step=field.type==='integer'?'1':'any';label.append(input);root.append(label);
  }
}
$('robotAdapter').onchange=()=>{renderInterfaceFields();managementControls();};
$('saveInterface').onclick=()=>action(async()=>{
  const options={};for(const input of document.querySelectorAll('[data-interface-field]'))options[input.dataset.interfaceField]=input.type==='checkbox'?input.checked:input.type==='number'?Number(input.value):input.value.trim();
  const r=await api('/api/settings/save',{revision:settingsData.revision,group:'interface',robot_adapter:$('robotAdapter').value,options});renderSettings(r.settings);toast('接口已保存，待应用');
});
function renderSettingsFeedback(){
  if(!$('settingsJoints'))return;const root=$('settingsJoints');root.replaceChildren();root.className='panel-body settings-joint-grid';
  const desc=status.capabilities?.robot;if(!desc)return;
  for(const module of desc.modules){const group=node('div','settings-joint-group');group.append(node('h3','',module.label));
    for(const id of module.channels){const index=desc.channels.findIndex(c=>c.id===id),c=desc.channels[index],v=status.feedback_valid?status.state?.[index]:null;
      const line=node('div','settings-joint-line');line.append(node('span','',c.label),node('strong','',Number.isFinite(v)?(c.unit==='rad'?(v*180/Math.PI).toFixed(2)+'°':v.toFixed(4)+' '+c.unit):'—'));group.append(line);}
    if(module.channels.some(id=>desc.channels.find(c=>c.id===id)?.angle_basis==='encoder'))group.append(node('small','muted','编码器角度 · 零位待标定'));
    root.append(group);
  }
}
function cameraAction(key,health){
  const group=node('div','camera-device-actions');
  for(const operation of ['reconnect','release']){const button=node('button','small-button');button.dataset.cameraAction=key;button.dataset.operation=operation;
    button.setAttribute('aria-label',cameraLabel(key)+'相机'+(operation==='release'?'释放':'启用'));
    button.onclick=()=>action(async()=>{activeCameraOperation={key,operation};updateCameraActions();try{const result=await api('/api/cameras/'+operation,{camera:key});if(result.connected===false)throw Error(result.message||result.health?.[key]?.error);toast(result.message);}finally{activeCameraOperation=null;updateCameraActions();}});
    setCameraAction(button,health);group.append(button);}
  return group;
}
function setCameraAction(button,h){
  const release=button.dataset.operation==='release',working=activeCameraOperation?.key===button.dataset.cameraAction&&activeCameraOperation?.operation===button.dataset.operation;button.textContent=working?(release?'释放中…':'启用中…'):release?'释放':'启用';
  button.disabled=busy||!connected||['PREPARING','RECORDING','EXPORTING'].includes(status.phase)||(status.writer_queue||0)>0||(release?h?.error==='相机已释放':!!h?.healthy);
  button.title=busy?'设备操作中':!connected?'等待服务连接':['PREPARING','RECORDING','EXPORTING'].includes(status.phase)||(status.writer_queue||0)>0?'结束录制并等待写入后可操作':release?(h?.error==='相机已释放'?'已释放，不占用设备':'停止取流并释放设备'):(h?.healthy?'已启用':'连接相机并开始取流');
}
function updateCameraActions(){for(const badge of document.querySelectorAll('[data-camera-status]')){const h=status.health?.cameras?.[badge.dataset.cameraStatus];badge.textContent=h?.error==='相机已释放'?'已释放':h?.healthy?'已启用':'未就绪';badge.className='badge '+(h?.healthy?'green':'');}
  for(const button of document.querySelectorAll('[data-camera-action]')){
  const h=status.health?.cameras?.[button.dataset.cameraAction];setCameraAction(button,h);
  const row=button.closest('.camera-config-row');if(row){const badge=row.querySelector('.camera-config-status');badge.textContent=h?.error==='相机已释放'?'已释放':h?.healthy?'在线':h?'离线':'待应用';}}
}
function addCameraRow(key='',binding={}){
  const row=node('div','camera-config-row');row.savedBinding=structuredClone(binding);row.dataset.originalKey=key;
  const fields=[['id','标识',key,'text'],['label','名称',binding.label||cameraLabel(key),'text'],['driver','驱动',binding.driver||'realsense','select'],['serial','序列号',binding.serial||'','text'],['width','宽',binding.width||848,'number'],['height','高',binding.height||480,'number'],['fps','帧率',binding.fps||60,'number']];
  for(const [name,label,value,type] of fields){const wrap=node('label','field',label),input=node(type==='select'?'select':'input');input.dataset.cameraField=name;
    if(type==='select'){input.add(new Option('RealSense','realsense'));input.add(new Option('Orbbec','orbbec'));}else{input.type=type;if(type==='number'){input.min=1;input.step=1;}}
    input.value=value;wrap.append(input);row.append(wrap);}
  const actions=node('div','camera-config-actions'),badge=node('span','badge camera-config-status','待应用');actions.append(badge);
  if(status.capabilities?.cameras.some(c=>c.id===key))actions.append(cameraAction(key,status.health?.cameras?.[key]));
  const remove=node('button','quiet','移除');remove.type='button';remove.onclick=()=>row.remove();remove.dataset.managementAction='camera-remove';actions.append(remove);row.append(actions);$('cameraSettings').append(row);updateCameraActions();
}
$('addCamera').onclick=()=>{addCameraRow();managementControls();};
$('saveCameraBindings').onclick=()=>action(async()=>{
  const bindings={};for(const row of document.querySelectorAll('.camera-config-row')){
    const values={};for(const input of row.querySelectorAll('[data-camera-field]'))values[input.dataset.cameraField]=input.type==='number'?Number(input.value):input.value.trim();
    const id=values.id;delete values.id;if(!/^[a-z][a-z0-9_]{0,63}$/.test(id))throw Error('标识需以小写字母开头，仅含字母、数字和下划线');
    if(bindings[id])throw Error('相机标识重复');const previous=row.savedBinding.driver===values.driver?row.savedBinding:{};bindings[id]={...previous,...values};
  }
  const r=await api('/api/settings/save',{revision:settingsData.revision,group:'cameras',camera_bindings:bindings});renderSettings(r.settings);toast('相机配置已保存，待应用');
});

function feedbackValue(packet,channel){
  let value=packet;for(const key of channel.packet_path||[]){if(value==null)return null;value=value[key];}
  return Number.isFinite(value)?value:null;
}
function renderHands(){
  const root=$('handCards'),groups=status.capabilities?.robot?.end_effectors||[];
  $('handPanel').hidden=replaying||!groups.length;if(replaying)return;
  const packet=status.feedback_valid?status.robot_feedback:null;
  const signature=JSON.stringify(groups);
  if(root.dataset.signature!==signature){root.dataset.signature=signature;root.replaceChildren();
    for(const group of groups){const card=node('section','hand-card');card.append(node('div','hand-kind',group.kind==='gripper'?'GRIPPER':group.kind==='dexterous_hand'?'DEXTEROUS HAND':'END EFFECTOR'),node('h3','',group.label));
      const values=node('div','hand-values');for(const channel of group.channels||[]){const cell=node('div','hand-value');cell.append(node('span','',channel.label),node('strong','','—'));cell.lastChild.dataset.handChannel=group.id+':'+channel.id;values.append(cell);}card.append(values,node('p','help',group.note||'原始反馈'));root.append(card);}
  }
  let received=0,total=0;for(const group of groups)for(const channel of group.channels||[]){total++;const v=feedbackValue(packet,channel),el=[...root.querySelectorAll('[data-hand-channel]')].find(e=>e.dataset.handChannel===group.id+':'+channel.id);if(el)el.textContent=v===null?'—':v.toFixed(3);if(v!==null)received++;}
  $('handStatus').textContent=received===total&&total?'原始反馈已收到':'等待反馈';$('handStatus').className='badge '+(received===total&&total?'green':'amber');
}
function renderEndEffectorSettings(){
  const slots=settingsData?.end_effector_slots||[],root=$('endEffectorChoices');root.replaceChildren();$('endEffectorSettings').hidden=!slots.length;
  for(const slot of slots){const label=node('label','field',slot.label),select=node('select');select.dataset.effectorSlot=slot.id;for(const o of slot.options)select.add(new Option(o.label,o.id));select.value=settingsData.end_effector_config?.[slot.id]||'none';label.append(select);root.append(label);}
}
$('saveEndEffectors').onclick=()=>action(async()=>{const choices={};for(const select of document.querySelectorAll('[data-effector-slot]'))choices[select.dataset.effectorSlot]=select.value;const r=await api('/api/settings/save',{group:'end_effectors',revision:settingsData.revision,choices});renderSettings(r.settings);toast('末端配置已保存，待应用');});

let motorDrafts={},motorDraftKey='',motorDisplayUnit='rad';
function renderMotorParameters(){
  const desc=status.capabilities?.robot,select=$('motorModule');if(!desc)return;select.replaceChildren();
  const identity=JSON.stringify([desc?.id,desc?.channels?.map(c=>[c.id,c.unit])]);
  const key='collector:motor-draft:v2:'+identity;
  if(key!==motorDraftKey){motorDraftKey=key;motorDrafts={};motorDisplayUnit='rad';try{const saved=JSON.parse(localStorage.getItem(key)||'{}');if(saved.schema==='si_v1'){motorDrafts=saved.channels||{};motorDisplayUnit=saved.angular_display==='deg'?'deg':'rad';}}catch{motorDrafts={};}const legacy='collector:motor-draft:v1:'+JSON.stringify([desc?.id,desc?.channels?.map(c=>c.id)]);try{if(localStorage.getItem(legacy))$('motorDraftStatus').textContent='旧草稿未定义单位，已保留，请按新单位重新填写';}catch{}}
  $('motorUnit').value=motorDisplayUnit;
  $('motorUnit').disabled=false;select.disabled=false;$('saveMotorDraft').disabled=false;
  for(const module of desc?.modules||[])select.add(new Option(module.label,module.id));renderMotorRows();
}
function motorUnits(channel){if(['rad','deg','°'].includes(channel.unit))return {unit:motorDisplayUnit==='deg'?'°':'rad',canonical:'rad',factor:motorDisplayUnit==='deg'?Math.PI/180:1};if(channel.unit==='m')return {unit:'m',canonical:'m',factor:1};return null;}
function renderMotorRows(){
  const desc=status.capabilities?.robot,module=desc?.modules.find(m=>m.id===$('motorModule').value),root=$('motorRows');root.replaceChildren();
  const channels=desc?.channels.filter(c=>module?.channels.includes(c.id))||[],units=[...new Set(channels.map(c=>motorUnits(c)?.unit||'接口单位待定义'))];
  $('motorSpeedHeading').textContent='速度'+(units.length===1?' · '+units[0]+'/s':' · 见各行');$('motorAccelerationHeading').textContent='加速度'+(units.length===1?' · '+units[0]+'/s²':' · 见各行');
  for(const channel of channels){const row=node('tr'),u=motorUnits(channel);tableCell(row,channel.label+(channel.motor_id!==undefined?' · '+channel.motor_id:''));
    for(const kind of ['speed','acceleration']){const input=node('input');input.type='number';input.step='any';input.placeholder='待设置';input.dataset.motorChannel=channel.id;input.dataset.motorParameter=kind;input.setAttribute('aria-label',channel.label+(kind==='speed'?'速度草稿':'加速度草稿'));
      input.min='0';input.disabled=!u;input.title=u?u.unit+(kind==='speed'?'/s':'/s²'):'此通道尚未声明标准运动单位';
      const saved=motorDrafts[channel.id]?.[kind];input.value=u&&Number.isFinite(saved)?String(Number((saved/u.factor).toPrecision(12))):'';
      input.oninput=()=>{motorDrafts[channel.id]={...motorDrafts[channel.id],unit:u.canonical,[kind]:input.value===''?null:Number(input.value)*u.factor};$('motorDraftStatus').textContent='草稿未保存 · 未下发';};const cell=node('div');cell.append(input);if(units.length>1)cell.append(node('small','help',input.title));tableCell(row,cell);}
    root.append(row);
  }
}
$('motorModule').onchange=renderMotorRows;
$('motorUnit').onchange=()=>{motorDisplayUnit=$('motorUnit').value;renderMotorRows();$('motorDraftStatus').textContent='草稿未保存 · 未下发';};
$('saveMotorDraft').onclick=()=>{try{for(const values of Object.values(motorDrafts))for(const kind of ['speed','acceleration']){const v=values[kind];if(v!=null&&(!Number.isFinite(v)||v<0))throw Error('请输入非负有限数值');}localStorage.setItem(motorDraftKey,JSON.stringify({schema:'si_v1',angular_display:motorDisplayUnit,channels:motorDrafts}));$('motorDraftStatus').textContent='草稿已保存 · 未下发';toast('已保存本机草稿');}catch(e){toast('保存失败：'+e.message);}};

let segmentData=null,segmentKey='',segmentLoading=false,segmentGeneration=0;
const segmentDrafts=new Map();
function segmentEpisode(){return status.phase==='RECORDING'?status.episode_id:(replaying||status.last_episode);}
function segmentControls(){
  const ep=segmentEpisode(),live=status.phase==='RECORDING',locked=busy||!connected,ready=segmentData?.episode_id===ep;
  const key=(ep||'')+':'+status.phase;if(key!==segmentKey){segmentKey=key;void loadSegments();}
  $('segmentScope').textContent=live?'当前录制':replaying?'当前回放':ep?'最近片段':'';
  $('markSegment').textContent=live?'标记分界':'在当前帧分段';
  $('markSegment').disabled=locked||segmentLoading||!ready||(!live&&(!replaying||!replayRow||replayPlaying||replayLoading));
  $('segmentLabel').disabled=$('markSegment').disabled;
  $('liveSegmentBar').hidden=!live;$('quickMarkSegment').disabled=$('markSegment').disabled;$('quickStopSegment').disabled=locked;
  $('refreshSegments').disabled=locked||!ep||segmentLoading;$('downloadSegments').disabled=locked||!ready||live;
  $('prepareNext').hidden=status.phase!=='REVIEW'||ep!==status.last_episode;
  $('prepareNext').disabled=locked||segmentLoading;
  $('segmentHint').textContent=live?'标记从下一采样帧开始；名称可后补。':replaying&&replayPlaying?'暂停播放后，可在当前帧分段。':ep?'回放定位后分段；移除分界会并入前段。':'开始录制后可打标记；结束本段后可命名并继续。';
  for(const button of document.querySelectorAll('[data-segment-action]'))button.disabled=locked||segmentLoading||!ready;
  if(live&&ready){const count=status.frames||0;for(const el of document.querySelectorAll('[data-segment-range]')){const i=Number(el.dataset.segmentRange),m=segmentData.marks[i],end=Math.min(count,segmentData.marks[i+1]?.frame??count);el.textContent=end>m.frame?m.frame+'–'+(end-1)+' 帧 · '+((end-m.frame)/segmentData.fps).toFixed(2)+' s':'等待下一帧';}}
}
async function loadSegments(){
  const ep=segmentEpisode(),generation=++segmentGeneration;segmentLoading=true;$('segmentError').textContent='';
  try{const data=ep?await api('/api/segments?episode='+encodeURIComponent(ep)):null;if(generation!==segmentGeneration)return;segmentData=data?.marks?data:null;renderSegments();}
  catch(e){if(generation!==segmentGeneration)return;segmentData=null;renderSegments();$('segmentError').textContent=e.message;}
  finally{if(generation===segmentGeneration){segmentLoading=false;segmentControls();}}
}
function renderSegments(){
  const root=$('segmentRows');root.replaceChildren();if(!segmentData)return;
  segmentData.marks.forEach((mark,i)=>{const row=node('div','segment-row'),info=node('div'),range=node('small','help');range.dataset.segmentRange=i;const end=Math.min(segmentData.frame_count,segmentData.marks[i+1]?.frame??segmentData.frame_count);range.textContent=end>mark.frame?mark.frame+'–'+(end-1)+' 帧 · '+((end-mark.frame)/segmentData.fps).toFixed(2)+' s':'等待下一帧';info.append(node('strong','','子任务 '+(i+1)),range);
    const input=node('input');input.type='text';input.maxLength=160;input.placeholder='待标注';input.setAttribute('aria-label','子任务 '+(i+1)+' 名称');const draftKey=segmentData.episode_id+':'+mark.id;input.value=segmentDrafts.get(draftKey)??mark.label;input.oninput=()=>segmentDrafts.set(draftKey,input.value);
    const save=node('button','small-button','保存名称');save.dataset.segmentAction='rename';save.onclick=()=>action(async()=>{await changeSegment({operation:'rename',mark_id:mark.id,label:input.value});segmentDrafts.delete(draftKey);toast('子任务名称已保存');});
    row.append(info,input,save);if(mark.id!=='start'){const remove=node('button','small-button quiet','移除分界');remove.dataset.segmentAction='delete';remove.onclick=()=>action(async()=>{await changeSegment({operation:'delete',mark_id:mark.id});toast('已并入前段');});row.append(remove);}root.append(row);
  });
}
async function changeSegment(change){const ep=segmentEpisode();if(segmentData?.episode_id!==ep)throw Error('请先刷新分段');const result=await api('/api/segments',{episode_id:ep,revision:segmentData.revision,...change});if(segmentEpisode()===ep){segmentData=result;renderSegments();}return result;}
$('markSegment').onclick=()=>action(async()=>{const live=status.phase==='RECORDING';await changeSegment({operation:'mark',label:$('segmentLabel').value,...(live?{}:{frame:Number($('replaySlider').value)})});$('segmentLabel').value='';toast('分界已保存，名称可稍后填写');});
$('quickMarkSegment').onclick=()=>$('markSegment').click();
$('quickStopSegment').onclick=()=>$('stop').click();
$('refreshSegments').onclick=()=>loadSegments();
$('downloadSegments').onclick=()=>action(async()=>{const ep=segmentEpisode(),data=await api('/api/segments?episode='+encodeURIComponent(ep));if(data.recording)throw Error('请结束本段后导出');const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'})),a=node('a');a.href=url;a.download=ep+'_segments.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('prepareNext').onclick=()=>action(async()=>{await api('/api/prepare-next',{episode_id:status.last_episode});status.phase='IDLE';live();toast('上一轮保持待审核，可开始下一轮');});
