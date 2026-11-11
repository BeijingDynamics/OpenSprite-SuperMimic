"""Rate conversion preserves all actuator fields and rejects disguised changes."""
import sys
from pathlib import Path
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from transport_profile import apply_all_joint_500hz, require_rate_only_change


class ProfileTests(unittest.TestCase):
    def test_rate_only_and_negative_guard_changes(self):
        with tempfile.TemporaryDirectory() as temp:
            old=Path(temp)/'old.tsv';new=Path(temp)/'new.tsv'
            original='motor_name\tpoll_rate_hz\tcap\n'+''.join(f'm{i}\t50\t3.50\n' for i in range(31))
            old.write_text(original);new.write_text(original)
            apply_all_joint_500hz(new)
            require_rate_only_change(old,new)
            self.assertEqual(new.read_text().replace('\t500\t','\t50\t'),original)
            new.write_text(new.read_text().replace('3.50','3.51',1))
            with self.assertRaises(ValueError): require_rate_only_change(old,new)
