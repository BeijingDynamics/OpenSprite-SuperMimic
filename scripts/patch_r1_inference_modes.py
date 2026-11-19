"""Expose bounded CPU-only ORT alternatives for diagnosing remaining scheduling tails."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=root/'outputs/deploy-sm-walk-r1-fast-20261119/source'
def edit(name,a,b):
    p=s/name;t=p.read_text()
    if t.count(a)!=1:raise ValueError(name+': '+a)
    p.write_text(t.replace(a,b),newline='\n')
edit('native/src/inference_cpu_layout.hpp',
     'if(threads!=2)throw std::runtime_error("isolated inference requires exactly two ORT threads");',
     'if(threads!=1 && threads!=2)throw std::runtime_error("isolated inference supports one or two ORT threads");')
edit('native/src/inference_cpu_layout.hpp',
     'if(workers.size()!=1)throw std::runtime_error("unexpected ORT worker count for isolated inference");\n  pin_tid(workers[0],5);pin_tid(getpid(),4);return workers[0];',
     'if(workers.size()!=static_cast<std::size_t>(threads-1))throw std::runtime_error("unexpected ORT worker count for isolated inference");\n'
     '  if(!workers.empty())pin_tid(workers[0],5);\n  pin_tid(getpid(),4);return workers.empty()?0:workers[0];')
edit('native/src/inference_cpu_layout.hpp',
     'out<<"{\\\"main_cpu\\\":4,\\\"ort_worker_cpu\\\":5,\\\"io_cpu\\\":1,\\\"threads\\\":[";',
     'out<<"{\\\"main_cpu\\\":4,\\\"ort_worker_cpu\\\":"<<(ort?5:-1)<<",\\\"inference_threads\\\":"<<(ort?2:1)<<",\\\"io_cpu\\\":1,\\\"threads\\\":[";')
edit('native/src/sprite_supermimic_replay.cpp','isolated_inference=false;int threads=1',
     'isolated_inference=false,no_ort_spinning=false;int threads=1')
edit('native/src/sprite_supermimic_replay.cpp','else if(option=="--isolated-inference")isolated_inference=true;',
     'else if(option=="--isolated-inference")isolated_inference=true;\n      else if(option=="--no-ort-spinning")no_ort_spinning=true;')
edit('native/src/sprite_supermimic_replay.cpp',
     'Ort::SessionOptions options;options.SetIntraOpNumThreads(threads);options.SetInterOpNumThreads(1);',
     'Ort::SessionOptions options;options.SetIntraOpNumThreads(threads);options.SetInterOpNumThreads(1);\n'
     '    if(no_ort_spinning)options.AddConfigEntry("session.intra_op.allow_spinning","0");')
p=s/'native/src/sprite_supermimic_replay.cpp';t=p.read_text()
t=t.replace('Inference affinity verified: main CPU 4, ORT worker CPU 5, sensor/gamepad IO CPU 1.',
            'Inference affinity verified; exact thread placement is recorded in the affinity report.')
p.write_text(t,newline='\n')
print('Optional inference variants added; control model and PD math unchanged.')
