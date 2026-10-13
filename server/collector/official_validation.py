"""Read exported data using the unmodified, pinned official LeRobot loader."""
from pathlib import Path


def validate_with_lerobot(root,horizon=20):
    import importlib.metadata
    import numpy as np
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from .storage import load_json

    version=importlib.metadata.version('lerobot')
    if version!='0.3.3': raise ValueError('Compatibility validation requires LeRobot 0.3.3.')
    root=Path(root);info=load_json(root/'meta/info.json')
    ds=LeRobotDataset('local/mock_bimanual',root=root,video_backend='pyav',
                     delta_timestamps={'action':[i/info['fps'] for i in range(horizon)]})
    assert len(ds)==info['total_frames']
    indices=set()
    for ep in range(info['total_episodes']):
        lo=int(ds.episode_data_index['from'][ep]);hi=int(ds.episode_data_index['to'][ep])
        indices.update((lo,(lo+hi)//2,hi-1))
    for i in sorted(indices):
        sample=ds[i]
        assert tuple(sample['observation.state'].shape)==(12,)
        assert tuple(sample['action'].shape)==(horizon,16)
        assert sample['action'].dtype==torch.float32
        for key in (name.removeprefix('observation.images.') for name in info['features'] if name.startswith('observation.images.')):
            image=sample['observation.images.'+key]
            assert tuple(image.shape)==(3,info['features']['observation.images.'+key]['shape'][0],
                                            info['features']['observation.images.'+key]['shape'][1])
            assert torch.isfinite(image).all() and image.min()>=0 and image.max()<=1
        assert sample['task'] in ds.meta.task_to_task_index
    for ep in range(info['total_episodes']):
        last=int(ds.episode_data_index['to'][ep])-1
        sample=ds[last]
        assert int((~sample['action_is_pad']).sum())==1
    return {'passed':True,'loader_version':version,'samples_checked':len(indices),
            'total_frames':len(ds),'action_chunk_shape':[horizon,16],
            'episode_boundary_padding_verified':True,'video_backend':'pyav'}
