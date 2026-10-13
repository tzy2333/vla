'use strict';
let taskListState="loading",captureTaskCreation=false,taskLoadPromise=null;
let managedTasks=[],settingsData=null,editingTask=null,managementLoading=false,managementPendingTab=null,resetRequestId=null;
const taskStateNames={active:'进行中',completed:'已完成',archived:'已归档'};
const jointNames=['肩前抬','肩外摆','大臂旋转','肘关节','腕旋转'];
const percent=x=>Number.isFinite(x)?x.toFixed(1)+'%':'—';
const count=x=>Number.isFinite(x)?x.toLocaleString('zh-CN'):'—';
function tableCell(row,value,cls){const td=node('td',cls);if(value instanceof Node)td.append(value);else td.textContent=value;row.append(td);return td;}

function renderCaptureTaskHelp(){
  const select=$('captureTask'),active=managedTasks.filter(t=>t.status==='active');
  const empty=taskListState==='ready'&&!active.length;
  select.hidden=empty;
  $('captureTaskHint').textContent=taskListState==='loading'?'正在加载任务列表…':taskListState==='error'?'任务列表加载失败，请重新加载；不关联任务仍可独立采集。':empty?(managedTasks.length?'暂无进行中的任务，本轮为独立采集。可新建任务，或在管理任务中调整状态。':'尚未创建任务，本轮为独立采集。需要汇总进度时，可先新建任务。'):select.value?'本轮将计入所选任务的进度；轮内阶段请使用子任务分段。':'不选则独立采集；选择任务后，本轮计入该任务进度。';
  $('retryCaptureTasks').hidden=taskListState!=='error';
}
function loadManagedTasks(){
  if(taskLoadPromise)return taskLoadPromise;
  taskListState='loading';renderCaptureTaskHelp();
  taskLoadPromise=(async()=>{
    try{
      const data=await api('/api/tasks');
      if(!Array.isArray(data.tasks))throw Error('任务列表格式无效');
      managedTasks=data.tasks;
      for(const id of ['captureTask','statsTask']){
        const select=$(id),value=select.value;
        // Preserve the current recording's selection while refreshing task metadata.
        if(id==='captureTask'&&!['IDLE','REVIEW'].includes(status.phase)&&value)continue;
        select.replaceChildren(new Option(id==='captureTask'?'不关联任务（独立采集）':'全部任务',''));
        for(const task of managedTasks)if(id==='statsTask'||task.status==='active')select.add(new Option(task.name,task.task_id));
        if([...select.options].some(o=>o.value===value))select.value=value;
      }
      taskListState='ready';renderTasks();
    }catch(e){taskListState='error';throw e;}
    finally{taskLoadPromise=null;renderCaptureTaskHelp();controls();}
  })();
  return taskLoadPromise;
}
function renderTasks(){
  const search=$('taskSearch').value.trim().toLowerCase(),filter=$('taskStatusFilter').value;
  const tasks=managedTasks.filter(t=>(filter==='all'||t.status===filter)&&(!search||[t.name,t.project,t.scene,t.object,t.operator,t.task_id].join(' ').toLowerCase().includes(search)));
  const rows=$('taskRows');rows.replaceChildren();$('taskEmpty').hidden=tasks.length>0;
  for(const task of tasks){
    const row=node('tr'),title=node('div');title.append(node('strong','',task.name),node('span','muted',task.task_id));tableCell(row,title);
    const project=node('div');project.append(node('strong','',task.project||'未分项目'),node('span','muted',[task.scene,task.object].filter(Boolean).join(' / ')||'未设场景'));tableCell(row,project);
    const progress=node('div');progress.append(node('strong','',task.metrics.finalized+' / '+task.target_episodes+' 轮'),node('span','muted','已保留 '+task.metrics.kept+' 轮'));const track=node('div','task-progress'),bar=node('i');bar.style.width=Math.min(100,task.metrics.finalized/task.target_episodes*100)+'%';track.append(bar);progress.append(track);tableCell(row,progress);
    tableCell(row,String(task.metrics.pending));const success=node('div');success.append(node('strong','',percent(task.metrics.task_success_rate)),node('span','muted',task.metrics.successes+' / '+task.metrics.labeled+' 人工标注'));tableCell(row,success);
    tableCell(row,node('span','task-badge '+task.status,taskStateNames[task.status]));const buttons=node('div','row-actions');
    const record=node('button','small-button','去采集');record.dataset.managementAction='record';record.dataset.active=String(task.status==='active');record.disabled=task.status!=='active'||!connected||busy;record.onclick=()=>{setTab('capture');$('captureTask').value=task.task_id;chooseCaptureTask();};
    const edit=node('button','small-button quiet','编辑');edit.dataset.managementAction='edit';edit.disabled=!connected||busy;edit.onclick=()=>openTask(task);buttons.append(record,edit);tableCell(row,buttons);rows.append(row);
  }
}
function chooseCaptureTask(){renderCaptureTaskHelp();const task=managedTasks.find(t=>t.task_id===$('captureTask').value);if(task){$('task').value=task.name;if(task.operator)$('operator').value=task.operator;}controls();}
function openTask(task=null){editingTask=task;$('taskDialogTitle').textContent=task?'编辑任务':'新建任务';for(const [id,key] of [['newTaskName','name'],['newTaskProject','project'],['newTaskScene','scene'],['newTaskObject','object'],['newTaskOperator','operator'],['newTaskNotes','notes']])$(id).value=task?.[key]||'';$('newTaskTarget').value=task?.target_episodes||100;$('newTaskStatus').value=task?.status||'active';$('taskDialog').showModal();}
$('newTask').onclick=()=>{captureTaskCreation=false;openTask();};
$('createCaptureTask').onclick=()=>{captureTaskCreation=true;openTask();};
$('manageCaptureTasks').onclick=()=>setTab('tasks');
$('retryCaptureTasks').onclick=()=>loadManagedTasks().catch(e=>toast(e.message));$('cancelTaskDialog').onclick=$('closeTaskDialog').onclick=()=>$('taskDialog').close();
$('taskForm').onsubmit=e=>{e.preventDefault();void action(async()=>{const body={name:$('newTaskName').value,project:$('newTaskProject').value,scene:$('newTaskScene').value,object:$('newTaskObject').value,operator:$('newTaskOperator').value,notes:$('newTaskNotes').value,target_episodes:Number($('newTaskTarget').value),status:$('newTaskStatus').value};if(editingTask)Object.assign(body,{task_id:editingTask.task_id,revision:editingTask.revision});const saved=await api('/api/tasks/save',body);$('taskDialog').close();await loadManagedTasks();if(captureTaskCreation&&saved.status==='active'&&['IDLE','REVIEW'].includes(status.phase)){$('captureTask').value=saved.task_id;chooseCaptureTask();}captureTaskCreation=false;toast('任务已保存');});};
$('taskSearch').oninput=renderTasks;$('taskStatusFilter').onchange=renderTasks;$('captureTask').onchange=chooseCaptureTask;
$('refreshTasks').onclick=()=>loadManagedTasks().catch(e=>toast(e.message));

async function loadDashboard(){
  const params=new URLSearchParams({day:$('statsDay').value,task_id:$('statsTask').value});
  const data=await api('/api/dashboard?'+params),s=data.summary;
  renderQualityOverview(data);
  $('overviewRounds').textContent=count(s.finalized);$('overviewSecondary').textContent='正常 '+count(s.completed)+' · 故障 '+count(s.faults);$('statsUpdated').textContent='更新于 '+data.generated_at.slice(11,19);
  const chart=$('dailyChart');chart.replaceChildren();const max=Math.max(1,...data.daily.map(d=>d.metrics.total));
  if(!data.daily.length)chart.append(node('p','muted','当前筛选下没有真实采集记录。'));
  for(const day of data.daily){const row=node('div','chart-row'),track=node('div','chart-track'),bar=node('div','chart-fill');bar.style.width=day.metrics.total/max*100+'%';track.append(bar);row.append(node('span','',day.day.slice(4,6)+'月'+day.day.slice(6)+'日'),track,node('strong','',String(day.metrics.total)));chart.append(row);}
  const faults=$('faultReasons');faults.replaceChildren();if(!data.fault_reasons.length)faults.append(node('p','muted','当前筛选下没有记录到故障。'));
  for(const f of data.fault_reasons.slice(0,8)){const row=node('div','fault-item');row.append(node('span','',f.reason.length>100?f.reason.slice(0,100)+'…':f.reason),node('strong','',String(f.count)));faults.append(row);}
}
const qualityLabels={completion_rate:'采集完成率',postprocess_success_rate:'后处理通过率',retention_rate:'旧版保留率',task_success_rate:'旧版任务标签成功率'};
let qualitySettings=null;
function renderQualityOverview(data){
  const q=data.quality,s=data.summary,root=$('statCards');root.replaceChildren();
  for(const [key,label] of Object.entries(qualityLabels)){const r=q?.rates?.[key],card=node('div','stat-card quality-card'),head=node('div','quality-card-head'),state=r?.state||'unavailable';card.dataset.quality=key;card.dataset.state=state;
    head.append(node('h3','',label),node('span','quality-state '+state,({pass:'达标',below:'低于阈值',insufficient:'样本不足',no_data:'无有效样本',unavailable:'暂无回传'})[state]));const value=node('strong','quality-number',Number.isFinite(r?.value)?r.value.toFixed(1):'—');if(Number.isFinite(r?.value))value.append(node('small','','%'));
    card.append(head,value,node('p','quality-denominator',r&&state!=='unavailable'?count(r.numerator)+' / '+count(r.denominator)+' 轮有效判定':'等待处理流程回传'),node('p','quality-target',r?'目标 ≥ '+r.threshold+'% · 至少 '+r.minimum_rounds+' 轮':'尚无可用统计'));root.append(card);}
  const flow=$('legacyPipelineCards');flow.replaceChildren();
  function stage(title,pairs,note,ready=true){const card=node('section','panel pipeline-card');card.append(node('h3','',title));const grid=node('div','pipeline-counts');for(const [label,value] of pairs){const cell=node('div');cell.append(node('span','',label),node('strong','',ready?count(value):'—'));grid.append(cell);}card.append(grid,node('p','help',note));flow.append(card);}
  stage('录制',[['已结束',s.finalized],['正常完成',s.completed],['故障中断',s.faults],['样本总数',s.frames]],'时长 '+(s.seconds/60).toFixed(1)+' 分钟 · 手动结束每轮');
  const upload=q?.pipeline?.upload,post=q?.pipeline?.postprocess,screen=q?.pipeline?.screening;
  stage('上传',[['完成',upload?.passed],['进行中',upload?.running],['失败',upload?.failed],['数据异常',upload?.data_error]],upload?.connected?'已回传 '+upload.reported+' 轮 · 未回传 '+upload.unreported+' 轮':'未接入上传流程',!!upload?.connected);
  stage('后处理',[['通过',post?.passed],['处理中',post?.running],['失败',post?.failed],['数据异常',post?.data_error]],post?.connected?'已回传 '+post.reported+' 轮 · 未回传 '+post.unreported+' 轮':'未接入后处理流程',!!post?.connected);
  stage('旧版保留记录',[['通过',s.kept],['未通过',s.reviewed-s.kept],['待审核',s.pending],['筛查异常',screen?.connected?(screen.failed+screen.data_error):null]],screen?.connected?'筛查已回传 '+screen.reported+' 轮':'筛查异常暂无回传');
  qualitySettings=q?.thresholds||null;const fields=$('qualityThresholdFields');fields.replaceChildren();
  if(qualitySettings){for(const [key,label] of [...Object.entries(qualityLabels),['minimum_rounds','最少有效轮次']]){const field=node('label','field',label+(key==='minimum_rounds'?'':'（%）')),input=node('input');input.type='number';input.required=true;input.dataset.qualityThreshold=key;input.min=key==='minimum_rounds'?'1':'0';input.max=key==='minimum_rounds'?'1000000':'100';input.step=key==='minimum_rounds'?'1':'0.1';input.value=key==='minimum_rounds'?qualitySettings.minimum_rounds:qualitySettings.rates[key];field.append(input);fields.append(field);}$('thresholdSummary').textContent='最少 '+qualitySettings.minimum_rounds+' 轮有效样本';}
  $('saveQualityThresholds').disabled=!qualitySettings||busy||!connected;
}
$('qualityThresholdForm').onsubmit=e=>{e.preventDefault();if(!qualitySettings)return;void action(async()=>{const values=Object.fromEntries([...document.querySelectorAll('[data-quality-threshold]')].map(el=>[el.dataset.qualityThreshold,Number(el.value)])),minimum=values.minimum_rounds;delete values.minimum_rounds;await api('/api/quality/thresholds',{revision:qualitySettings.revision,minimum_rounds:minimum,rates:values});await loadDashboard();$('qualityThresholdStatus').textContent='阈值已保存，立即生效';toast('质量阈值已保存');});};
function refreshDashboard(){void loadDashboard().catch(e=>{$('statsUpdated').textContent='更新失败：'+e.message;toast(e.message);});}
$('refreshDashboard').onclick=refreshDashboard;$('statsDay').onchange=$('statsTask').onchange=refreshDashboard;
$('clearStatsFilter').onclick=()=>{$('statsDay').value='';$('statsTask').value='';refreshDashboard();};

function modelInput(id,value,kind='number'){const input=node('input');input.id=id;input.type=kind;input.value=value;if(kind==='number'){input.step='any';input.min='-2';input.max='2';}return input;}
function numericInput(id){const input=$(id);if(!input.value.trim()||!Number.isFinite(Number(input.value)))throw Error('模型参数不能为空且必须是有效数字。');return Number(input.value);}
function renderRFModel(data){
  const rows=$('jointRows');rows.replaceChildren();
  for(const side of ['left','right']){
    const a=data.model.arms[side],c=a.encoder_to_model,divider=node('tr','joint-divider');const cell=tableCell(divider,side==='left'?'左臂':'右臂');cell.colSpan=9;rows.append(divider);
    for(let j=0;j<5;j++){
      const row=node('tr');tableCell(row,jointNames[j]+' · '+a.motor_ids[j]);
      for(const key of ['angle','speed','torque']){const td=tableCell(row,'—','joint-value');td.id=side+'_'+key+'_'+j;}
      const zero=modelInput(side+'_zero_'+j,c.zero_raw[j]);zero.min='-1';zero.max='1';tableCell(row,zero);
      const sign=node('select');sign.id=side+'_sign_'+j;sign.add(new Option('+1','1'));sign.add(new Option('−1','-1'));sign.value=String(c.sign[j]);tableCell(row,sign);
      for(let axis=0;axis<3;axis++)tableCell(row,modelInput(side+'_origin_'+j+'_'+axis,a.joints[j].origin_m[axis]));rows.append(row);
    }
  }
  ['tcpX','tcpY','tcpZ'].forEach((id,i)=>$(id).value=data.model.tcp_translation_m[i]);
  $('calibrationStatus').textContent=['left','right'].every(s=>data.model.arms[s].encoder_to_model.zero_calibration_verified)?'当前零偏来自已完成的零位采样。':'当前参数含人工修改，未重新进行零位采样标定。';
  const controller=$('controllerRows');controller.replaceChildren();for(const p of data.controller_parameters){const row=node('tr');[p.name,String(p.code),'未接入参数读取','缺少完整报文，暂不可写'].forEach(value=>tableCell(row,value));controller.append(row);}
  updateJointTable();managementControls();
}
function updateJointTable(){
  renderSettingsFeedback();if(!settingsData?.model_supported||!settingsData.current_model)return;const packet=status.robot_feedback;
  for(const side of ['left','right'])for(let j=0;j<5;j++){
    const index=settingsData.current_model.arms[side].obs_indices[j],angle=packet?.joint_angles_rad?.[side]?.[j];
    $(side+'_angle_'+j).textContent=Number.isFinite(angle)?(angle*180/Math.PI).toFixed(3):'—';
    for(const [key,offset] of [['speed',20],['torque',40]]){const v=packet?.raw_observation?.[index+offset];$(side+'_'+key+'_'+j).textContent=Number.isFinite(v)?v.toFixed(5):'—';}
  }
}
async function loadSettings(){renderSettings(await api('/api/settings'));await loadControlHistory();}
async function loadControlHistory(){const data=await api('/api/robot/control-history'),root=$('controlHistory');root.replaceChildren();if(!data.operations.length)root.append(node('p','muted','尚未通过本平台发送本体操作。'));for(const r of data.operations){const item=node('div','control-entry');item.append(node('strong','',r.sent_at+' · '+r.name),node('p','',r.message));root.append(item);}}
function managementControls(){
  const locked=busy||!connected||status.phase==='PREPARING',recording=['PREPARING','RECORDING','EXPORTING'].includes(status.phase)||(status.writer_queue||0)>0;
  $('saveQualityThresholds').disabled=locked||!qualitySettings;
  for(const id of ['resetRobot','retryCameras','saveCaptureSettings','saveModelSettings','applySettings'])$(id).disabled=locked||recording||(id==='resetRobot'&&!status.health?.robot?.healthy)||(id!=='resetRobot'&&id!=='retryCameras'&&!settingsData);
  for(const id of ['newTask','saveTask'])$(id).disabled=locked;
  for(const button of document.querySelectorAll('[data-management-action]'))button.disabled=locked||(button.dataset.managementAction==='record'&&button.dataset.active!=='true');
  $('captureTask').disabled=locked||!!replaying||!['IDLE','REVIEW'].includes(status.phase)||taskListState==='loading'||(taskListState==='ready'&&!managedTasks.some(t=>t.status==='active'));
  $('createCaptureTask').disabled=locked||!['IDLE','REVIEW'].includes(status.phase);
  $('retryCaptureTasks').disabled=locked||taskListState==='loading';
  if($('captureTask').value)$('task').disabled=true;
  for(const input of document.querySelectorAll('#captureSettingsForm input,#captureSettingsForm select,#jointRows input,#jointRows select,.tcp-settings input'))input.disabled=locked||recording;
  if(settingsData?.model_supported===false)$('saveModelSettings').disabled=true;
  for(const button of document.querySelectorAll('#addCamera,#saveCameraBindings,[data-management-action="camera-remove"]'))button.disabled=locked||recording;
  for(const input of document.querySelectorAll('#cameraSettings input,#cameraSettings select,#interfaceFields input'))input.disabled=locked||recording;
  updateCameraActions();
  $('saveEndEffectors').disabled=locked||recording||!settingsData;
  for(const select of document.querySelectorAll('[data-effector-slot]'))select.disabled=locked||recording;
}
$('refreshSettings').onclick=()=>loadSettings().catch(e=>toast(e.message));
$('captureSettingsForm').onsubmit=e=>{e.preventDefault();void action(async()=>{const r=await api('/api/settings/save',{group:'capture',revision:settingsData.revision,name:$('robotName').value,location:$('robotLocation').value,fps:Number($('settingsFps').value),camera_profile:$('cameraProfile').value});renderSettings(r.settings);$('captureSettingsResult').textContent=r.message;toast('设置已保存，等待应用');});};
$('saveModelSettings').onclick=()=>{if(!window.confirm('保存后将改变后续采集所用的软件模型参数，历史片段不变，也不会写入电机。确认保存？'))return;void action(async()=>{const arms={};for(const side of ['left','right'])arms[side]={zero_raw:Array.from({length:5},(_,j)=>numericInput(side+'_zero_'+j)),sign:Array.from({length:5},(_,j)=>numericInput(side+'_sign_'+j)),origins_m:Array.from({length:5},(_,j)=>Array.from({length:3},(_,a)=>numericInput(side+'_origin_'+j+'_'+a)))};const r=await api('/api/settings/save',{group:'model',revision:settingsData.revision,tcp_translation_m:['tcpX','tcpY','tcpZ'].map(numericInput),arms});renderSettings(r.settings);$('modelSettingsResult').textContent=r.message;toast('模型参数已保存，等待应用');});};
$('applySettings').onclick=()=>{if(!window.confirm('应用已保存设置会重新连接相机与关节反馈，不发送运动指令。确认继续？'))return;void action(async()=>{const r=await api('/api/settings/apply',{revision:settingsData.revision});settingsData=null;rateInitialized=false;await loadSettings();toast(r.message);});};
$('retryCameras').onclick=()=>action(async()=>{const r=await api('/api/cameras/reconnect',{});toast(r.message);});
$('resetRobot').onclick=()=>{$('resetAcknowledged').checked=false;$('confirmReset').disabled=true;resetRequestId=null;$('resetDialog').showModal();};
$('resetAcknowledged').onchange=()=>{$('confirmReset').disabled=!$('resetAcknowledged').checked;};$('cancelReset').onclick=()=>$('resetDialog').close();
$('confirmReset').onclick=()=>{if(!$('resetAcknowledged').checked)return;resetRequestId=resetRequestId||Array.from(crypto.getRandomValues(new Uint8Array(16)),v=>v.toString(16).padStart(2,'0')).join('');$('resetDialog').close();void action(async()=>{try{const r=await api('/api/robot/reset',{request_id:resetRequestId,confirmed:true});$('robotActionResult').textContent=r.message;await loadControlHistory();toast(r.message);}catch(e){$('robotActionResult').textContent='本次请求未确认：'+e.message+' 请查看操作记录，不要连续重复发送。';throw e;}});};
async function refreshManagement(tab){managementPendingTab=tab;if(managementLoading)return;managementLoading=true;try{while(managementPendingTab){const requested=managementPendingTab;managementPendingTab=null;try{if(requested==='tasks'||requested==='dashboard'||requested==='capture'||taskListState!=='ready')await loadManagedTasks();if(requested==='dashboard')await loadDashboard();if(requested==='robot')await loadSettings();}catch(e){toast(e.message);}}}finally{managementLoading=false;}}
setInterval(()=>{if(document.hidden||!connected)return;if(currentTab==='dashboard')refreshDashboard();if(currentTab==='tasks')void loadManagedTasks().catch(()=>{});},15000);
void loadManagedTasks().catch(()=>{});
