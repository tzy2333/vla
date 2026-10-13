"""Custom 12-state / 16-absolute-TCP-action adapter. No ALOHA transforms.

Callable classes are usable in openpi transforms.Group. Normalization, image
resize, prompt/state tokenization and padding are supplied by openpi itself.
"""
from dataclasses import dataclass
import numpy as np

IMAGE_KEYS={'head':'base_0_rgb','left_wrist':'left_wrist_0_rgb'}


def rgb_uint8(image):
    x=np.asarray(image)
    if x.ndim!=3: raise ValueError('Expected RGB image.')
    if x.shape[0]==3 and x.shape[-1]!=3:x=np.transpose(x,(1,2,0))
    if x.shape[-1]!=3:raise ValueError('Expected three RGB channels.')
    if np.issubdtype(x.dtype,np.floating):
        if not np.isfinite(x).all() or x.min()<0 or x.max()>1:
            raise ValueError('Float images from LeRobot must be in [0,1].')
        x=np.rint(x*255).astype(np.uint8)
    elif x.dtype!=np.uint8:raise ValueError('Expected uint8 image or float [0,1].')
    return x


@dataclass(frozen=True)
class TabletopInputs:
    def __call__(self,data):
        state=np.asarray(data['state'],dtype=np.float32)
        if state.shape!=(12,) or not np.isfinite(state).all():raise ValueError('Expected actual state[12].')
        images={dest:rgb_uint8(data['images'][key]) for key,dest in IMAGE_KEYS.items()}
        masks={k:np.True_ for k in images}
        # The unused checkpoint slot is masked and never recorded as a camera.
        images['right_wrist_0_rgb']=np.zeros_like(images['left_wrist_0_rgb'])
        masks['right_wrist_0_rgb']=np.False_
        result={'state':state,'image':images,'image_mask':masks,
                'prompt':data['prompt']}
        if 'actions' in data:
            a=np.asarray(data['actions'],dtype=np.float32)
            if a.ndim!=2 or a.shape[1]!=16 or not np.isfinite(a).all():
                raise ValueError('Expected absolute TCP actions[H,16].')
            for start in (3,11):
                if not np.allclose(np.linalg.norm(a[:,start:start+4],axis=1),1,atol=1e-4):
                    raise ValueError('Quaternion targets must be normalized before statistics/training.')
            result['actions']=a
        return result


@dataclass(frozen=True)
class TabletopOutputs:
    """Apply after openpi has UNNORMALIZED the predicted 32-wide tensor."""
    def __call__(self,data):
        a=np.asarray(data['actions'],dtype=np.float32)
        if a.ndim!=2 or a.shape[1]<16:raise ValueError('Prediction must contain >=16 fields.')
        a=a[:,:16].copy()
        if not np.isfinite(a).all():raise ValueError('Non-finite prediction.')
        for start in (3,11):
            q=a[:,start:start+4];n=np.linalg.norm(q,axis=1,keepdims=True)
            if np.any(n<1e-6):raise ValueError('Invalid predicted quaternion.')
            a[:,start:start+4]=q/n
            for i in range(1,len(a)):
                if np.dot(a[i-1,start:start+4],a[i,start:start+4])<0:a[i,start:start+4]*=-1
        a[:,[7,15]]=np.clip(a[:,[7,15]],0,1)
        # The downstream robot controller must still validate five-axis reachability.
        return {'actions':a}
