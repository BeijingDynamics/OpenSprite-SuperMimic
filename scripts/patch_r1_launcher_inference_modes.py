"""Allow controlled no-enable comparisons before sealing a single final mode."""
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'outputs/deploy-sm-walk-r1-fast-20261119/source/tools/run_supermimic_commissioning.py'
t=p.read_text()
a='"--threads", "2", "--isolated-inference",'
assert t.count(a)==1
t=t.replace(a,'"--threads", os.environ.get("SPRITE_INFERENCE_THREADS","2"), "--isolated-inference", *(["--no-ort-spinning"] if os.environ.get("SPRITE_NO_ORT_SPINNING")=="1" else []),')
a='and inference_affinity.get("ort_worker_cpu") == 5 and inference_affinity.get("io_cpu") == 1)'
assert t.count(a)==1
t=t.replace(a,'and inference_affinity.get("ort_worker_cpu") == (-1 if os.environ.get("SPRITE_INFERENCE_THREADS")=="1" else 5) and inference_affinity.get("io_cpu") == 1)')
a='                    inference_affinity=inference_affinity,'
assert t.count(a)==1
t=t.replace(a,a+'\n                    inference_no_spinning=os.environ.get("SPRITE_NO_ORT_SPINNING")=="1",')
p.write_text(t,newline='\n')
