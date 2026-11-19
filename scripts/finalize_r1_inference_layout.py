"""Require the validated thread layout in the commissioning launcher and operator gate."""
from pathlib import Path

root=Path(__file__).resolve().parents[1]
src=root/'outputs/deploy-sm-walk-r1-fast-20261119/source'
p=src/'tools/run_supermimic_commissioning.py';t=p.read_text()
a='*(["--isolated-inference"] if os.environ.get("SPRITE_ISOLATED_INFERENCE")=="1" else [])'
assert t.count(a)==1;t=t.replace(a,'"--isolated-inference"')
a='    passed = not failure and disabled == 31 and enables == (31 if mode == "powered" else 0)'
assert t.count(a)==1
t=t.replace(a,'''    affinity_path = out/"native.bin.affinity.json"
    inference_affinity = json.loads(affinity_path.read_text()) if affinity_path.exists() else {}
    if not (inference_affinity.get("verified") is True and inference_affinity.get("main_cpu") == 4
            and inference_affinity.get("ort_worker_cpu") == 5 and inference_affinity.get("io_cpu") == 1):
        failure = failure or "verified inference thread layout evidence missing"
'''+a)
a='                    can_schedule=can_schedule,';assert t.count(a)==1
t=t.replace(a,a+'\n                    inference_affinity=inference_affinity,')
p.write_text(t,newline='\n')
p=src/'operator-xbox.sh';t=p.read_text()
a="    frequency = report.get('cpu_frequency', {})";assert t.count(a)==1
t=t.replace(a,"""    affinity = report.get('inference_affinity', {})
    if not (affinity.get('verified') is True and affinity.get('main_cpu') == 4
            and affinity.get('ort_worker_cpu') == 5 and affinity.get('io_cpu') == 1
            and report.get('feedback_torque_mode') == 'observe_only'
            and report.get('xbox_forward_acceleration_m_s2') == .10):
        raise SystemExit('Required inference layout and command/torque profile were not verified.')
"""+a)
a="echo 'Inference CPU 4/5 automatically hold 1344 MHz for this session and restore their prior settings on exit.'"
assert t.count(a)==1
t=t.replace(a,a+"\necho 'Inference main: CPU 4; ORT worker: CPU 5; sensor/gamepad receive threads: CPU 1. Startup verifies every thread.'")
p.write_text(t,newline='\n')
print('Final launcher requires verified per-thread affinity; operator evidence gate updated.')
