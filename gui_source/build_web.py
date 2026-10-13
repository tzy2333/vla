"""Assemble the existing acquisition UI and the management extension."""
from pathlib import Path
base=Path(__file__).parent
shell=(base/'gui_shell.html').read_text(encoding='utf-8')
client=(base/'gui_client.js').read_text(encoding='utf-8')
if 'id="captureTask"' not in shell:
    shell=shell.replace('<label class="field">任务描述', '<label class="field">关联任务<select id="captureTask"><option value="">不关联任务（填写下方描述）</option></select></label><label class="field">任务描述',1)
    shell=shell.replace('<label class="review-success"><input type="checkbox" id="success">任务成功（人工确认）</label>', '<label class="field" style="margin-top:12px">任务结果（人工标注）<select id="success"><option value="unknown">未标注</option><option value="success">成功</option><option value="failure">失败</option></select></label>')
    shell=shell.replace('机器人 · 模拟反馈','机器人 · 检查中')
    shell=shell.replace('<p class="help">10月12日复验：50Hz连续录制30秒、1507组通过；长期稳定性仍需验证。</p>','<p class="help">诊断基于当前配置的真实观测；历史低分辨率验收不能替代高分辨率复测。</p>')
    for new_id,label,before in [('dashboard','采集总览','capture'),('tasks','采集任务','library'),('robot','本体与设置','devices')]:
        marker='<button class="tab" role="tab" id="tab-'+before+'"'
        button=f'<button class="tab" role="tab" id="tab-{new_id}" aria-selected="false" aria-controls="{new_id}" data-tab="{new_id}" tabindex="-1">{label}</button>'
        shell=shell.replace(marker,button+marker,1)
    (base/'gui_shell.html').write_text(shell,encoding='utf-8')
if 'task_id:$(\'captureTask\').value' not in client:
    client=client.replace("if(name==='library')renderLibrary();}","if(name==='library')renderLibrary();if(typeof refreshManagement==='function')void refreshManagement(name);}",1)
    client=client.replace("renderSelection();}\nfunction updateDevices", "renderSelection();if(typeof managementControls==='function')managementControls();}\nfunction updateDevices",1)
    client=client.replace("$('success').checked=false", "$('success').value='unknown'")
    client=client.replace("success:$('success').checked", "success:$('success').value==='unknown'?null:$('success').value==='success'")
    client=client.replace("task:$('task').value,operator:", "task:$('task').value,task_id:$('captureTask').value||null,operator:")
    client=client.replace("if(!replaying)values(status.state,status.action,status.ee_pose);renderTiming();", "if(!replaying)values(status.state,status.action,status.ee_pose);if(typeof updateJointTable==='function')updateJointTable();renderTiming();")
    client=client.replace("+' · '+cam.native_fps+'fps'", "+' · '+cam.width+'×'+cam.height+' · '+cam.native_fps+'fps'")
    (base/'gui_client.js').write_text(client,encoding='utf-8')
css=(base/'gui_management.css').read_text(encoding='utf-8')+'\n'+(base/'gui_review.css').read_text(encoding='utf-8')
markup=(base/'gui_management.html').read_text(encoding='utf-8')+'\n'+(base/'gui_review.html').read_text(encoding='utf-8')
markup=markup.replace('<span class="help">速度与加速度可保存草稿，暂不下发。</span>', '<label class="field">角度单位<select id="motorUnit"><option value="rad">弧度 · rad</option><option value="deg">角度 · °</option></select></label><span class="help">标准单位保存草稿，接口接入后可下发。</span>')
markup=markup.replace('<th>速度 · 单位待定义</th><th>加速度 · 单位待定义</th>', '<th id="motorSpeedHeading">速度 · rad/s</th><th id="motorAccelerationHeading">加速度 · rad/s²</th>')
markup=markup.replace('等待电机参数接口、单位和范围','等待厂家参数接口与范围映射')
for control in ('motorModule','motorUnit','saveMotorDraft'):
    markup=markup.replace('id="'+control+'"','id="'+control+'" disabled',1)
markup=markup.replace('<details class="panel" id="motorParameters"', '''<div class="panel end-effector-settings" id="endEffectorSettings"><div class="panel-head"><h2>末端设备</h2></div><div class="panel-body"><div id="endEffectorChoices" class="settings-grid"></div><div class="end-effector-actions"><button id="saveEndEffectors">保存末端配置</button><p class="help">保存后，点击“应用已保存设置”生效。</p></div></div></div><details class="panel" id="motorParameters"''',1)
extra=(base/'gui_management.js').read_text(encoding='utf-8')
css+='\n'+(base/'gui_modular.css').read_text(encoding='utf-8')
extra+='\n'+(base/'gui_modular.js').read_text(encoding='utf-8')
shell=shell.replace('<div class="device-pill" id="headPill"><span class="dot"></span><span>头部相机 · 检查中</span></div><div class="device-pill" id="wristPill"><span class="dot"></span><span>左腕相机 · 检查中</span></div>','<div id="cameraPills" class="camera-pills"></div>')
shell=shell.replace('<span class="badge" id="phase">','<span id="robotModel" class="model-name"></span><span class="badge" id="phase">')
shell=shell.replace('<label class="field">关联任务', '''<fieldset class="module-select"><legend>关节模块</legend><div id="moduleChoices" class="module-options"></div></fieldset><fieldset class="module-select"><legend>相机</legend><div id="cameraChoices" class="module-options"></div></fieldset><p id="moduleNote" class="module-note"></p><label class="field">关联任务''',1)
shell=shell.replace('<div class="metrics">','''<div class="panel live-panel"><div class="panel-head"><h2>关节状态</h2><div><span id="jointAge" class="muted"></span> <span id="jointStatus" class="badge">等待反馈</span></div></div><div id="liveJoints" class="joint-grid"></div></div><div class="panel pull-summary"><div class="quality-head"><h2>关节拉取</h2><span id="pullScope" class="muted">本次录制</span></div><div class="pull-metrics"><div><small>成功率</small><strong id="pullRate">—</strong></div><div><small>有效响应频率</small><strong id="pullHz">—</strong></div></div><p id="pullDetail" class="help">开始录制后统计</p></div><div class="metrics">''',1)
markup=markup.replace('<button id="retryCameras"','<button id="reconnectFeedback">重连关节反馈</button><button id="retryCameras"',1)
markup=markup.replace('<button id="retryCameras"','<button id="releaseCameras">释放相机</button><button id="retryCameras"',1)
markup=markup.replace('<div id="activeConfiguration"', '<div class="robot-interface"><label class="field">机器人接口<select id="robotAdapter"></select></label><button id="saveInterface" class="small-button">保存接口</button></div><div id="interfaceFields" class="settings-grid"></div><div id="activeConfiguration"',1)
for old,new in [('双臂数采','机器人数采'),('检查画面，记录一段完整任务，再回放审核。','选择模块，采集与回放。'),('头部 + 左腕 · 固定底盘 · 时间对齐v1','模块化采集 · 实时监测'),('两路接收时间差','相机接收时间差'),('预览刷新频率独立于采集频率，数据按设定频率保存。','预览频率独立于录制。'),('真实数据、明确分母。看清采集质量与人工任务结果。','采集进度与数据质量。'),('让每一段采集都可追溯','采集概览'),('查看真实关节反馈，管理采集配置和后续计算所用的本体模型。','设备接口与采集参数。')]:
    shell=shell.replace(old,new);markup=markup.replace(old,new)
shell=shell.replace('<button class="tab" role="tab" id="tab-library"','<button class="tab" role="tab" id="tab-reviews" aria-selected="false" aria-controls="reviews" data-tab="reviews" tabindex="-1">审核中心</button><button class="tab" role="tab" id="tab-library"',1)
markup=markup.replace('<div class="overview-section-head"><h2>审核进度</h2>','<div id="dashboardReviewCounts" class="review-counts"></div><div class="overview-section-head"><h2>审核进度</h2>',1)
markup=markup.replace('<div id="cameraSettings">','<div class="controls"><button id="discoverCameras">发现接入相机</button><span id="inventoryStatus" class="help"></span></div><div id="inventoryList" class="inventory-list"></div><div id="cameraSettings">',1)
shell=shell.replace('<div class="work-grid">','<div id="discoveryNotice" class="camera-discovery-note" hidden><span id="discoveryText"></span><button id="configureDiscovered">配置新相机</button></div><div class="work-grid">',1)
extra+='\n'+(base/'gui_review.js').read_text(encoding='utf-8')
html=shell.replace('</style>',css+'\n</style>',1).replace('<footer class="footer">',markup+'\n<footer class="footer">',1)
import re
live_panel=re.search(r'<div class="panel live-panel">.*?<div id="liveJoints" class="joint-grid"></div></div>',html).group(0)
html=html.replace(live_panel,'',1)
hand_panel='''<section class="hand-panel" id="handPanel" aria-label="手部反馈"><div class="hand-heading"><h3>手部</h3><span id="handStatus" class="badge">等待反馈</span></div><div id="handCards" class="hand-cards"></div></section>'''
robot_panel=live_panel.replace('<h2>关节状态</h2>','<h2>机器人状态</h2>').replace('class="panel live-panel"','class="panel live-panel" id="robotStatePanel"')
robot_panel=robot_panel[:-6]+hand_panel+'</div>'
preview_start='<div class="view-column"><div class="panel"><div class="view-top">'
assert preview_start in html
html=html.replace(preview_start,'<div class="view-column"><div class="observation-grid"><div class="panel preview-panel"><div class="view-top">',1)
preview_end='<div class="panel pull-summary">'
assert preview_end in html
html=html.replace(preview_end,robot_panel+'</div>'+preview_end,1)
assert 'id="handPanel"' in html
html=html.replace('<label class="field">关联任务', '<label class="field" title="归入采集任务以统计进度；子任务在分段标注中设置。">归属任务（可选）')
html=html.replace('不关联任务（填写下方描述）','不关联任务（独立采集）')
html=html.replace('归属任务（可选）','关联任务（可不选）')
html=html.replace('<select id="captureTask">','<select id="captureTask" aria-describedby="captureTaskHint" disabled>')
html=html.replace('</select></label><label class="field">任务描述', '</select></label><div class="capture-task-help"><p id="captureTaskHint" class="help" role="status">正在加载任务列表…</p><div class="capture-task-actions"><button id="createCaptureTask" type="button" class="small-button quiet" disabled>新建任务</button><button id="manageCaptureTasks" type="button" class="small-button quiet">管理任务</button><button id="retryCaptureTasks" type="button" class="small-button quiet" hidden>重新加载</button></div></div><label class="field">任务描述',1)
html=html.replace('id="stop" disabled hidden>结束录制','id="stop" disabled hidden>结束本段')
segment_panel='''<section class="panel segment-panel" id="segmentPanel"><div class="panel-head"><h2>子任务分段</h2><span id="segmentScope" class="muted"></span></div><div class="panel-body"><div class="segment-toolbar"><input type="text" id="segmentLabel" maxlength="160" placeholder="下一子任务名称（可后补）" aria-label="下一子任务名称"><button id="markSegment">标记分界</button></div><p class="help" id="segmentHint">录制中打点，回放时补标。</p><div id="segmentRows" class="segment-rows"></div><div class="controls"><button id="refreshSegments" class="small-button">刷新分段</button><button id="downloadSegments" class="small-button">导出分段 JSON</button><button id="prepareNext" class="small-button" hidden>准备下一段</button></div><p id="segmentError" class="help" role="status"></p></div></section>'''
html=html.replace('<div class="panel pull-summary">',segment_panel+'<div class="panel pull-summary">',1)
html=html.replace('<div class="work-grid">','<section class="work-mode-bar" id="workModeBar" aria-label="当前工作模式"><div><strong id="workModeTitle">实时采集</strong><p id="workModeHint">开始时自动连接所选相机。</p></div><button id="returnToCapture" class="primary" type="button" hidden>返回采集，录制下一轮</button></section><div class="work-grid">',1)
html=html.replace('<div class="work-grid">','''<div class="recording-segment-bar" id="liveSegmentBar" hidden><span><strong>录制中</strong><small>分段不中断采集，名称可后补</small></span><div><button id="quickMarkSegment" class="primary">标记子任务分界</button><button id="quickStopSegment" class="stop">结束本段</button></div></div><div class="work-grid">''',1)
for old,new in [('结束本段','结束本轮'),('准备下一段','准备下一轮'),('目标保留片段数','目标录制轮次'),('<th>保留进度</th>','<th>录制轮次 / 目标</th>'),('<span class="step">1</span>录制任务','<span class="step">1</span>本轮录制')]:
    html=html.replace(old,new)
client=client.replace("},1000);setInterval", "},500);setInterval")
client=client.replace("rates[0]===rates[1]?rates[0]+' fps':rates.join('/')", "!rates.length?'—':rates.every(r=>r===rates[0])?rates[0]+' fps':rates.join('/')")
client=client.replace("catch(e){connected=false;$('connectionBadge')", "catch(e){connected=false;status.feedback_valid=false;status.state=null;status.robot_feedback=null;if(typeof renderLiveJoints==='function')renderLiveJoints();if(typeof renderHands==='function')renderHands();$('connectionBadge')")
assert html.count('<!--SCRIPT-->')==1
target=base/'server_src/web/index.html' if (base/'server_src').is_dir() else base.parent/'server/web/index.html'
target.write_text(html.replace('<!--SCRIPT-->',client+'\n'+extra),encoding='utf-8')
print('Management UI assembled.')
