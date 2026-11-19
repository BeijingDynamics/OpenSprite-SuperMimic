"""Apply reviewed, exact substitutions only to the isolated architecture experiment."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'outputs/actor-architecture-20261119/source'


def replace(path, before, after, count=1):
    target = SOURCE / path
    text = target.read_text(encoding='utf-8')
    if text.count(before) != count:
        raise ValueError(f'Unexpected source anchor count: {path}: {before[:80]}')
    target.write_text(text.replace(before, after), encoding='utf-8')


def main():
    replace('opensprite_tracker/supermimic/joint_interfaces.py',
        'def forward(self,history,command):return self.shared.decode(history,self.shared.velocity_latent(history,command))',
        'def forward(self,history,command):return self.shared.velocity_action_and_latent(history,command)[0]')
    replace('opensprite_tracker/supermimic/joint_learner.py',
        'latent=shared.velocity_latent(history,command)\n        action=shared.decode(history,latent).clamp(-20,20)',
        'action,latent=shared.velocity_action_and_latent(history,command)\n        action=action.clamp(-20,20)')
    for name, variable in [('train_joint.py','cfg'), ('evaluate_joint.py',"ck['config']"),
                           ('export_joint.py',"ck['config']")]:
        path = 'scripts/' + name
        target = SOURCE / path
        if not target.exists():
            target.write_text((ROOT / path).read_text(encoding='utf-8'), encoding='utf-8')
        text = target.read_text(encoding='utf-8')
        # Keep import ordering after the existing ROOT/sys.path bootstrap.
        anchor = 'from opensprite_tracker.supermimic.revisions import '
        position = text.index(anchor)
        text = text[:position] + 'from opensprite_tracker.supermimic.actor_architectures import make_controller\n' + text[position:]
        original = f"SharedController(hidden=tuple({variable}['network_hidden']),normalized_gelu=revision['normalized_gelu'])"
        if text.count(original) != 1:
            raise ValueError('Constructor anchor missing: ' + name)
        target.write_text(text.replace(original, f'make_controller({variable})'), encoding='utf-8')
    replace('scripts/export_joint.py',
        'latent=self.shared.velocity_latent(history,x[:,990:993])\n        return self.shared.decode(history,latent).clamp(-20,20)',
        'action,_=self.shared.velocity_action_and_latent(history,x[:,990:993])\n        return action.clamp(-20,20)')
    replace('scripts/export_joint.py', "(args.output/'contract.json').write_text",
        "contract['actor_architecture']=ck['config'].get('actor_architecture','full')\n    contract['direct_velocity_ablation']=shared.direct_velocity\n    (args.output/'contract.json').write_text")
    replace('scripts/train_joint.py',
        "shared.load_state_dict(old['shared']);world.load_state_dict(old['wm'])\n        reset_both=",
        "if cfg.get('actor_initialization')!='fresh_actor_common_world_v1':\n            raise ValueError('Architecture experiments require common fresh-actor initialization')\n        for name in ['obs_norm','motion_norm','command_norm']:\n            norm=getattr(shared,name)\n            norm.load_state_dict({key.removeprefix(name+'.'):value for key,value in old['shared'].items() if key.startswith(name+'.')})\n        world.load_state_dict(old['wm'])\n        reset_both=")
    replace('scripts/train_joint.py',
        "inherited=['shared controller and normalizers','world model']",
        "actor_initialization=cfg['actor_initialization'],inherited=['observation and goal normalizers','world model']")
    replace('scripts/train_joint.py',
        "reset=(['motion critic','velocity critic'] if reset_both else ['velocity critic'])",
        "reset=['all actor weights']+(['motion critic','velocity critic'] if reset_both else ['velocity critic'])")
    replace('scripts/train_joint.py',
        "for key in ['num_envs','hist','goal_frames','horizon','wm_width','network_hidden','command_ranges']:",
        "for key in ['num_envs','hist','goal_frames','horizon','wm_width','network_hidden','command_ranges','actor_architecture','actor_initialization']:")
    replace('scripts/train_joint.py',
        'shared_world_model=True,shared_decoder=True,',
        "shared_world_model=True,shared_decoder=not shared.direct_velocity,actor_architecture=cfg['actor_architecture'],direct_velocity_auxiliary_latent=shared.direct_velocity,")
    # Keep the original supervised-decoder gradient rule in every arm. The direct
    # head is owned exclusively by the velocity optimizer, including its auxiliary.
    print('Isolated architecture source prepared; original training source unchanged.')


if __name__ == '__main__':
    main()
