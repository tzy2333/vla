"""Starter config for a separate, provisioned openpi GPU environment.

Register build_config() in openpi.training.config._CONFIGS before get_config/CLI
enumeration. This module is not imported by the Jetson collection process.
"""
import dataclasses
from pathlib import Path
from openpi import transforms
from openpi.models import pi0_config
from openpi.training import config,weight_loaders
from training.pi05_adapter import TabletopInputs,TabletopOutputs


@dataclasses.dataclass(frozen=True)
class TabletopDataConfig(config.DataConfigFactory):
    def create(self,assets_dirs:Path,model_config):
        repack=transforms.Group(inputs=[transforms.RepackTransform({
            'images':{'head':'observation.images.head','left_wrist':'observation.images.left_wrist'},
            'state':'observation.state','actions':'action','prompt':'prompt'})])
        return dataclasses.replace(self.create_base_config(assets_dirs,model_config),
            repack_transforms=repack,
            data_transforms=transforms.Group(inputs=[TabletopInputs()],outputs=[TabletopOutputs()]),
            model_transforms=config.ModelTransformFactory()(model_config),
            action_sequence_keys=('action',),prompt_from_task=True)


def build_config(repo_id='local/tabletop'):
    model=pi0_config.Pi0Config(pi05=True,action_dim=32,action_horizon=50,discrete_state_input=True,
                              paligemma_variant='gemma_2b_lora',action_expert_variant='gemma_300m_lora')
    return config.TrainConfig(name='pi05_tabletop_eef_lora',model=model,
        data=TabletopDataConfig(repo_id=repo_id,assets=config.AssetsConfig(asset_id='tabletop_eef'),
                               base_config=config.DataConfig(prompt_from_task=True)),
        weight_loader=weight_loaders.CheckpointWeightLoader('gs://openpi-assets/checkpoints/pi05_base/params'),
        freeze_filter=model.get_freeze_filter(),ema_decay=None,batch_size=4,num_workers=2,
        num_train_steps=10000)
