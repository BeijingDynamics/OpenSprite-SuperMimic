"""Add an optional verified split layout for the real concurrent sensor workload."""
from pathlib import Path
import shutil

root=Path(__file__).resolve().parents[1]
src=root/'outputs/deploy-sm-walk-r1-fast-20261119/source'
shutil.copyfile(root/'scripts/inference_cpu_layout.hpp',src/'native/src/inference_cpu_layout.hpp')
def edit(name,a,b):
    p=src/name;t=p.read_text()
    if t.count(a)!=1:raise ValueError(name+': '+a)
    p.write_text(t.replace(a,b),newline='\n')
for name,anchor in [('policy_gamepad_linux.hpp','  GamepadSnapshot snapshot(){'),
                    ('policy_imu_client.hpp','    bool snapshot(ImuSnapshot& result) {')]:
    edit('native/src/'+name,'#pragma once','#pragma once\n#include "inference_cpu_layout.hpp"')
    edit('native/src/'+name,anchor,
         '    void pin_worker(int cpu) { sprite_schedule::pin_thread(worker_.native_handle(),cpu); }\n'+anchor)
name='native/src/sprite_supermimic_replay.cpp'
edit(name,'#include "inference_diagnostics.hpp"','#include "inference_diagnostics.hpp"\n#include "inference_cpu_layout.hpp"')
edit(name,'bool paced=false,diagnose_inference=false;', 'bool paced=false,diagnose_inference=false,isolated_inference=false;')
edit(name,'else if(option=="--diagnose-inference")diagnose_inference=true;',
     'else if(option=="--diagnose-inference")diagnose_inference=true;\n      else if(option=="--isolated-inference")isolated_inference=true;')
edit(name,'    const auto before_ort_threads=diagnose_inference?sprite_diagnostics::tids():std::vector<int>{};',
     '    const auto before_ort_threads=(diagnose_inference||isolated_inference)?sprite_diagnostics::tids():std::vector<int>{};')
edit(name,'    Ort::Session session(env,argv[1],options);',
     '    Ort::Session session(env,argv[1],options);\n'
     '    const int ort_worker_tid=isolated_inference?sprite_schedule::isolate_compute(before_ort_threads,threads):0;\n'
     '#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC\n'
     '    if(isolated_inference && gamepad_preview)gamepad_preview->pin_worker(1);\n'
     '#endif')
edit(name,'    sprite_policy::NativeImuClient imu(imu_socket,true,imu_kind=="icm");',
     '    sprite_policy::NativeImuClient imu(imu_socket,true,imu_kind=="icm");\n'
     '    if(isolated_inference)imu.pin_worker(1);')
anchor='    const double load_ms=ms(Clock::now()-load_start);'
edit(name,anchor,anchor+'\n#ifndef SPRITE_SUPERMIMIC_STATE_IPC\n'
     '    if(isolated_inference)sprite_schedule::verify_and_save(argv[3],ort_worker_tid,0);\n#endif')
anchor='    sprite_policy::ImuSnapshot sample;const auto wait_start=Clock::now();'
edit(name,anchor,
     '    if(isolated_inference) {\n'
     '      int io_threads=1;\n#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC\n'
     '      if(gamepad_preview)++io_threads;\n#endif\n'
     '      sprite_schedule::verify_and_save(argv[3],ort_worker_tid,io_threads);\n'
     '      std::cout<<"Inference affinity verified: main CPU 4, ORT worker CPU 5, sensor/gamepad IO CPU 1."<<std::endl;\n'
     '    }\n'+anchor)
name='tools/run_supermimic_commissioning.py'
anchor='"--threads", "2", *(["--diagnose-inference"]'
edit(name,anchor,'"--threads", "2", *(["--isolated-inference"] if os.environ.get("SPRITE_ISOLATED_INFERENCE")=="1" else []), *(["--diagnose-inference"]')
print('Optional compute/IO isolation prepared; scheduling priority and CPU clocks unchanged.')
