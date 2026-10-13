"""Versioned, read-only robot descriptors and per-episode channel selection."""
import copy
import importlib.metadata
import re

SCHEMA = 'modular_robot_observation_v1'


def available_adapters():
    return [{'id':'rf_udp_20010','label':'RF 双臂 · UDP 20010'}] + [
        {'id':e.name,'label':e.name} for e in importlib.metadata.entry_points(group='collector.robot_adapters')
        if e.name!='rf_udp_20010']


def interface_fields(name):
    """Installed adapter factories may declare typed configuration_fields.

    Plugin values are passed under config['adapter_options'][adapter_id].
    No module path or executable code is accepted from the web UI.
    """
    if name=='rf_udp_20010':
        return [dict(key='host',label='控制器地址',type='text',default='192.168.11.100'),
                dict(key='port',label='请求端口',type='integer',default=3000,min=1,max=65535),
                dict(key='receive_port',label='反馈端口',type='integer',default=3002,min=1,max=65535),
                dict(key='poll_hz',label='读取频率（Hz）',type='number',default=50,min=1,max=100),
                dict(key='timeout_ms',label='响应超时（ms）',type='number',default=100,min=10,max=300)]
    matches=list(importlib.metadata.entry_points(group='collector.robot_adapters',name=name))
    if len(matches)!=1:raise ValueError('机器人接口尚未安装。')
    fields=copy.deepcopy(getattr(matches[0].load(),'configuration_fields',[]))
    if not isinstance(fields,list):raise ValueError('Invalid adapter configuration schema.')
    keys=set()
    for f in fields:
        if not isinstance(f,dict) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',f.get('key','')) or f.get('type') not in ('text','integer','number','boolean'):
            raise ValueError('Invalid adapter field.')
        if f['key'] in keys:raise ValueError('Duplicate adapter field.')
        keys.add(f['key'])
    return fields


def validate_interface_options(name, values):
    import math
    fields=interface_fields(name)
    if not isinstance(values,dict) or set(values)-{f['key'] for f in fields}:
        raise ValueError('接口参数不匹配。')
    result={}
    for f in fields:
        v=values.get(f['key'],f.get('default')); kind=f['type']
        valid=(isinstance(v,str) and 0<len(v.strip())<=200 if kind=='text' else
               type(v) is bool if kind=='boolean' else
               type(v) is int if kind=='integer' else type(v) in (int,float))
        if not valid:raise ValueError('接口参数无效：'+f['key'])
        if kind in ('integer','number') and (not math.isfinite(v) or v<f.get('min',-1e30) or v>f.get('max',1e30)):
            raise ValueError('接口参数超出范围：'+f['key'])
        result[f['key']]=v.strip() if kind=='text' else v
    return result


def create_robot(config, default_factory):
    name = config.get('robot_adapter', 'rf_udp_20010')
    if name == 'rf_udp_20010':
        return default_factory(config)
    matches = list(importlib.metadata.entry_points(group='collector.robot_adapters', name=name))
    if len(matches) != 1:
        raise ValueError('Robot adapter is not installed or is ambiguous: ' + str(name))
    robot = matches[0].load()(config)
    try:
        if robot.is_mock or robot.supports_commands:
            raise ValueError('Acquisition adapters must be real and read-only.')
        for method in ('describe','live_snapshot','health_check','calibration_snapshot','interpolate','project_packet','close'):
            if not callable(getattr(robot,method,None)):
                raise ValueError('Adapter method missing: '+method)
        if not getattr(robot,'session',None) or not getattr(robot,'pull_statistics',None):
            raise ValueError('Adapter must expose a session ID and request statistics.')
        validate_descriptor(robot.describe())
    except Exception:
        robot.close()
        raise
    return robot


def validate_descriptor(value):
    d = copy.deepcopy(value)
    if not isinstance(d, dict) or d.get('version') != 1:
        raise ValueError('Unsupported robot descriptor.')
    if not all(isinstance(d.get(k),str) and d[k] for k in ('id','label','source')):
        raise ValueError('Robot descriptor requires id, label and source.')
    channels, modules = d.get('channels'), d.get('modules')
    if not isinstance(channels, list) or not isinstance(modules, list):
        raise ValueError('Robot must describe channels and modules.')
    ids = [c['id'] for c in channels]
    groups = [g['id'] for g in modules]
    for ident in ids + groups:
        if not isinstance(ident, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', ident):
            raise ValueError('Invalid channel/module ID.')
    if len(set(ids)) != len(ids) or len(set(groups)) != len(groups):
        raise ValueError('Duplicate channel or module IDs.')
    for c in channels:
        if type(c.get('index')) is not int or c['index'] < 0 or not c.get('unit'):
            raise ValueError('Each channel needs an index and unit.')
    if len({c['index'] for c in channels}) != len(channels):
        raise ValueError('Duplicate state indices.')
    claimed = []
    for group in modules:
        if not group.get('channels') or not set(group['channels']) <= set(ids):
            raise ValueError('Module references unknown channels.')
        claimed.extend(group['channels'])
    if sorted(claimed) != sorted(ids):
        raise ValueError('Every channel must belong to exactly one module.')
    return d


def selection(descriptor, camera_keys, requested=None):
    groups = [m['id'] for m in descriptor['modules']]
    if requested is None:
        requested = {'modules': groups, 'cameras': list(camera_keys)}
    if not isinstance(requested, dict) or set(requested) != {'modules', 'cameras'}:
        raise ValueError('Choose modules and cameras.')
    for key, allowed in [('modules', groups), ('cameras', camera_keys)]:
        chosen = requested[key]
        if not isinstance(chosen, list) or any(not isinstance(x, str) for x in chosen):
            raise ValueError('Invalid capture selection.')
        if len(set(chosen)) != len(chosen) or not set(chosen) <= set(allowed):
            raise ValueError('Unknown or duplicate capture module: ' + key)
    if not requested['modules'] and not requested['cameras']:
        raise ValueError('至少选择一个关节模块或相机。')
    # Descriptor order is authoritative; click order must not reorder joint vectors.
    return {'modules': [g for g in groups if g in requested['modules']],
            'cameras': [k for k in camera_keys if k in requested['cameras']]}


def selected_channels(descriptor, chosen):
    ids = {c for g in descriptor['modules'] if g['id'] in chosen['modules'] for c in g['channels']}
    return [copy.deepcopy(c) for c in descriptor['channels'] if c['id'] in ids]


def project_state(state, channels):
    return [state[c['index']] for c in channels]


def camera_bindings(config):
    bindings = {}
    if config.get('head_camera_serial'):
        bindings['head'] = dict(config['head_camera_profile'], serial=config['head_camera_serial'], driver='orbbec')
    for key, settings in config.get('wrist_cameras', {}).items():
        bindings[key] = dict(settings, driver='realsense')
    if 'camera_bindings' in config:
        bindings = copy.deepcopy(config['camera_bindings'])
    serials = []
    for key, settings in bindings.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', key):
            raise ValueError('Invalid camera ID.')
        if settings.get('driver') not in ('orbbec', 'realsense') or not settings.get('serial'):
            raise ValueError('Camera needs a supported driver and stable serial.')
        serials.append(str(settings['serial']))
        for field in ('width','height','fps'):
            if type(settings.get(field)) is not int or not 1<=settings[field]<= (240 if field=='fps' else 8192):
                raise ValueError('Invalid camera profile: '+field)
    if len(set(serials)) != len(serials):
        raise ValueError('One physical camera cannot be bound twice.')
    return bindings
