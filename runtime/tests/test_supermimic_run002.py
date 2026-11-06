"""Test the new model/profile boundary independently of hardware I/O."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.supermimic_run002 import validate_binding, ASSET_REVISION, PROFILE_SHA256

class Run002BindingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.profile=self.root/'profile.json'
        self.profile.write_text(json.dumps({'hardware_qualified':False,'model_binding':{
            'asset_revision':ASSET_REVISION,'asset_directory':'models/v5',
            'files_sha256':{'models/v5/sprite0825_float.urdf':'urdf-sha'}},
            'joints':{'joint':dict(kp=130,kd=5,effort_nm=40,default_pos_rad=.15,velocity_rad_s=9.3)}}))
        self.data={'training_run':'joint-waist-v5-002','asset_revision':ASSET_REVISION,
            'control_profile_sha256':PROFILE_SHA256,'asset_urdf_sha256':'urdf-sha','hardware_qualified':False,
            'joint_names':['joint'],'stiffness':[130],'damping':[5],'effort_limit':[40],
            'default_joint_pos':[.15],'action_offset':[.15],'velocity_limit':[9.3],'action_scale':[.25]}

    def check(self,data):
        with patch('open_sprite_runtime.supermimic_run002.checked_model'),patch(
            'open_sprite_runtime.supermimic_run002.file_sha256',return_value=PROFILE_SHA256):
            return validate_binding(PolicyContract(self.root/'contract.json',data),self.profile)

    def test_accepts_named_profile_with_explicit_500hz_override(self):
        result=self.check(self.data)
        self.assertEqual(result['effective_training_pd_hz'],500)
        self.assertFalse(result['hardware_qualified'])

    def test_rejects_stale_model_and_profile_identity(self):
        for key,value in [('training_run','run001'),('asset_revision','old'),('control_profile_sha256','old'),
                          ('asset_urdf_sha256','old')]:
            with self.subTest(key=key):
                data=copy.deepcopy(self.data);data[key]=value
                with self.assertRaises(ValueError):self.check(data)

    def test_rejects_silent_gain_cap_offset_and_cost_threshold_edits(self):
        for key in ['stiffness','damping','effort_limit','default_joint_pos','action_offset','velocity_limit','action_scale']:
            with self.subTest(key=key):
                data=copy.deepcopy(self.data);data[key][0]+=.01
                with self.assertRaises(ValueError):self.check(data)

    def test_rejects_hardware_qualification_in_training_artifact(self):
        data=copy.deepcopy(self.data);data['hardware_qualified']=True
        with self.assertRaises(ValueError):self.check(data)
