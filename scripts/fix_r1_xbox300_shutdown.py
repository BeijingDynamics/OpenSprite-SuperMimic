"""Close controller IPC after disable verification, before long evidence writes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('stage', type=Path)
parser.add_argument('--build', action='store_true')
args = parser.parse_args()
stage = args.stage
path = stage/'native/src/sprite_can_shadow.cpp'
text = path.read_text()
anchor = '  std::ostringstream report;\n  report << std::fixed << std::setprecision(6)\n      << "{\\n  \\\"mode\\\": \\\"" << (powered ? "native_suspended_protected_policy_actuation"'
assert text.count(anchor) == 1
text = text.replace(anchor,
    '  // Disable verification is complete. Release the actor before writing long traces.\n'
    '  // The supervisor still requires the CAN report and all 31 disabled readbacks.\n'
    '  close_ipc(ipc);\n\n'+anchor)
path.write_text(text, newline='\n')
test = stage/'native/tests/test_bounded_preview.cpp'
text = test.read_text()
anchor = '  using sprite_policy::sling_notice_elapsed;'
assert text.count(anchor) == 1
test.write_text(text.replace(anchor, '''  {
    int pair[2];
    require(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, pair) == 0, "completion socket pair");
    IpcSocket completed;
    completed.client_fd = pair[0];
    completed.state_sequence = 16301;
    completed.target_count = 16301;
    close_ipc(completed);
    char byte = 0;
    require(recv(pair[1], &byte, 1, MSG_DONTWAIT) == 0,
            "actor receives EOF before controller trace serialization");
    require(completed.state_sequence == 16301 && completed.target_count == 16301,
            "IPC completion preserves timing evidence counters");
    close_ipc(completed);
    close(pair[1]);
  }
'''+anchor), newline='\n')
if args.build:
    subprocess.run(['cmake', '--build', str(stage/'build'), '-j2', '--target',
                    'sprite_can_shadow', 'test_bounded_preview'], check=True)
    subprocess.run(['ctest', '--test-dir', str(stage/'build'), '--output-on-failure'], check=True)
    manifest_path = stage/'build-manifest.json'
    (stage/'build-manifest-before-ipc-shutdown.json').write_text(manifest_path.read_text())
    manifest = json.loads(manifest_path.read_text())
    for name in manifest['files']:
        manifest['files'][name] = hashlib.sha256((stage/name).read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    print('Shutdown ordering fix compiled and tested. No hardware session started.')
