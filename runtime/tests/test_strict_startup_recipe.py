from pathlib import Path
import unittest,tempfile,hashlib,json,importlib.util

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('strict_startup_reproducer',ROOT/'tools/reproduce_strict_startup.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class StrictStartupRecipeTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)
        self.contract=self.root/'contract.json';self.contract.write_text(json.dumps({'policy_onnx_sha256':'model'}))
        self.hardware=self.root/'hardware.json';self.hardware.write_text(json.dumps({'controller':{'damiao_embedded_kd_max':3}}))
        self.scene=self.root/'scene.mjb';self.scene.write_bytes(b'frozen scene')
        p=json.loads((ROOT/'config/supermimic_run002_strict_startup.experiment.json').read_text())
        for key,path in [('contract_sha256',self.contract),('hardware_sha256',self.hardware),('scene_sha256',self.scene)]:p[key]=hashlib.sha256(path.read_bytes()).hexdigest()
        p['policy_sha256']='model';self.profile=self.root/'profile.json';self.profile.write_text(json.dumps(p))
        self.commands=self.root/'commands.json';self.commands.write_text(json.dumps([{'command':[.25,0,-.15]}]))

    def arguments(self):return module.checked_arguments(self.profile,self.contract,self.hardware,self.scene,self.commands,self.root/'out','pressure',[101])

    def test_reproducer_always_uses_strict_gate(self):
        args=self.arguments();self.assertIn('runtime_strict',args);self.assertIn('--damping-compensation',args)
        self.assertEqual(args[args.index('--handoff-seconds')+1],'1.5')

    def test_rejects_changed_protection_inventory(self):
        self.hardware.write_text(json.dumps({'controller':{'damiao_embedded_kd_max':5}}))
        with self.assertRaisesRegex(ValueError,'frozen input'):self.arguments()

    def test_rejects_untested_command_instead_of_silent_clamping(self):
        self.commands.write_text(json.dumps([{'command':[.3,0,-.15]}]))
        with self.assertRaisesRegex(ValueError,'outside'):self.arguments()

    def test_rejects_early_motion_during_startup_prefix(self):
        self.commands.write_text(json.dumps([{'command':[.25,0,-.15],
            'stages':[{'at_seconds':1.,'command':[.25,0,-.15]}]}]))
        with self.assertRaisesRegex(ValueError,'startup prefix'):self.arguments()
