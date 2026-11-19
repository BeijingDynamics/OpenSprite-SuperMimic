"""Add optional in-process timing without changing control semantics."""
from pathlib import Path
import shutil

root=Path(__file__).resolve().parents[1]
base=root/'outputs/deploy-sm-walk-r1-fast-20261119'
source=base/'source'
shutil.copyfile(base/'source.tar.gz',base/'source-before-live-diagnostics.tar.gz')
shutil.copyfile(root/'scripts/inference_diagnostics.hpp',source/'native/src/inference_diagnostics.hpp')
p=source/'native/src/sprite_supermimic_replay.cpp'
t=p.read_text()
def edit(a,b):
    global t
    if t.count(a)!=1:raise ValueError(a)
    t=t.replace(a,b)
edit('#include <onnxruntime_cxx_api.h>','#include <onnxruntime_cxx_api.h>\n#include "inference_diagnostics.hpp"\n#include <memory>')
edit('  std::size_t completed_frames=0;','  std::size_t completed_frames=0;\n  std::unique_ptr<sprite_diagnostics::Probe> inference_probe;')
edit('    bool paced=false;int threads=1,warmup=50;', '    bool paced=false,diagnose_inference=false;int threads=1,warmup=50;')
edit('      if(option=="--benchmark")paced=true;', '      if(option=="--benchmark")paced=true;\n      else if(option=="--diagnose-inference")diagnose_inference=true;')
edit('    Ort::Session session(env,argv[1],options);',
     '    const auto before_ort_threads=diagnose_inference?sprite_diagnostics::tids():std::vector<int>{};\n'
     '    Ort::Session session(env,argv[1],options);\n'
     '    if(diagnose_inference) inference_probe=std::make_unique<sprite_diagnostics::Probe>(argv[3],count,before_ort_threads);')
edit('      record.obs=core.observe(inputs[i]);const auto before_infer=Clock::now();',
     '      record.obs=core.observe(inputs[i]);\n      if(inference_probe) inference_probe->begin(i);\n      const auto before_infer=Clock::now();')
edit('      cycle.infer_end_ns=sprite_policy::monotonic_ns();\n#endif',
     '      cycle.infer_end_ns=sprite_policy::monotonic_ns();\n#endif\n      if(inference_probe) inference_probe->end();')
p.write_text(t,newline='\n')
p=source/'tools/run_supermimic_commissioning.py';t=p.read_text()
anchor='"--threads", "2", "--warmup", "100", "--state-socket", state_socket'
assert t.count(anchor)==1
t=t.replace(anchor,'"--threads", "2", *(["--diagnose-inference"] if os.environ.get("SPRITE_INFERENCE_DIAGNOSTICS")=="1" else []), "--warmup", "100", "--state-socket", state_socket')
p.write_text(t,newline='\n')
print('Optional live timing instrumentation prepared.')
