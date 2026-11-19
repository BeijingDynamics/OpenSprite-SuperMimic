"""Bind the operator entry to the validated two-thread no-idle-spinning mode."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=root/'outputs/deploy-sm-walk-r1-fast-20261119/source'
p=s/'operator-xbox.sh';t=p.read_text()
a='export LD_LIBRARY_PATH="$stage/build"';assert t.count(a)==1
t=t.replace(a,a+'\nexport SPRITE_INFERENCE_THREADS=2\nexport SPRITE_NO_ORT_SPINNING=1\nunset SPRITE_INFERENCE_DIAGNOSTICS')
t=t.replace('fixed-disabled-01','sleeping-disabled-01').replace('fixed-fault-01','sleeping-fault-01')
a="and report.get('xbox_forward_acceleration_m_s2') == .10)";assert t.count(a)==1
t=t.replace(a,"and report.get('xbox_forward_acceleration_m_s2') == .10\n            and report.get('inference_no_spinning') is True)")
a="echo 'Inference main: CPU 4; ORT worker: CPU 5; sensor/gamepad receive threads: CPU 1. Startup verifies every thread.'"
assert t.count(a)==1
t=t.replace(a,a+"\necho 'ORT idle spinning is disabled. Model outputs and the 50 Hz policy rate are unchanged.'")
p.write_text(t,newline='\n')
p=s/'run-disabled.py';t=p.read_text()
a="env={**os.environ,'PYTHONPATH':str(STAGE/'runtime/src'),'LD_LIBRARY_PATH':str(STAGE/'build')}"
assert t.count(a)==1
t=t.replace(a,"env={**os.environ,'PYTHONPATH':str(STAGE/'runtime/src'),'LD_LIBRARY_PATH':str(STAGE/'build'),\n         'SPRITE_INFERENCE_THREADS':'2','SPRITE_NO_ORT_SPINNING':'1','SPRITE_INFERENCE_DIAGNOSTICS':'0'}")
p.write_text(t,newline='\n')
p=root/'scripts/collect_r1_fast_validation.py';t=p.read_text()
t=t.replace('fixed-disabled-01','sleeping-disabled-01').replace('fixed-disabled-02','sleeping-disabled-02').replace('fixed-fault-01','sleeping-fault-01')
# The comparison reference remains the prior qualified 120-second deployment.
t=t.replace("OLD/'sleeping-disabled-01/commissioning.json'","OLD/'fixed-disabled-01/commissioning.json'")
t=t.replace("assert report['feedback_torque_mode']=='observe_only'", "assert report['inference_no_spinning'] is True\n        assert report['feedback_torque_mode']=='observe_only'")
t=t.replace("members += [p.name for p in STAGE.glob('failed-*')]", "members += [p.name for p in STAGE.glob('failed-*')]\n    members += [p.name for p in STAGE.glob('fixed-disabled-*')]\n    members += [p.name for p in STAGE.glob('single-disabled-*')]\n    members += ['inference-mode-parity']")
t=t.replace("'Timestamp driver registers echo skb before publishing TX and cleans failed submissions.'", "'Timestamp driver registers echo skb before publishing TX and cleans failed submissions.',\n                                  'Two ORT threads with idle spinning disabled; policy recurrence unchanged.'")
p.write_text(t,newline='\n')
