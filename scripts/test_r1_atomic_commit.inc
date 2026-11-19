// Exercise real mapping and encoding at every phase of the cached preview cycle.
void test_atomic_commit() {
  for (std::size_t phase=1; phase<=10; ++phase) {
    Fixture cached, fresh;
    for (auto* f : {&cached,&fresh}) {
      const char* names[]={"left_ankle_motor_a","left_ankle_motor_b",
                           "right_ankle_motor_a","right_ankle_motor_b"};
      for (std::size_t i=0;i<4;++i) {
        f->motors[27+i].name=names[i];
        f->kinematics.joint_names[27+i]="ankle_joint_"+std::to_string(i);
      }
      for (std::size_t i=0;i<kJointCount;++i) {
        f->ipc.latest_target.kp[i]=20.;
        f->ipc.latest_target.kd[i]=1.;
        f->ipc.latest_target.position_rad[i]=.05;
      }
      preview_final_motor_commands(f->motors,f->ipc,f->kinematics,false,true,true);
      for (std::size_t i=0;i<kJointCount;++i)
        f->ipc.latest_target.position_rad[i]=.10+.001*i;
    }
    cached.ipc.motor_preview_count=phase;
    // Reproduce the old stale direct-drive cache independently of the fix.
    preview_final_motor_commands(cached.motors,cached.ipc,cached.kinematics,false,true,false);
    if (phase%10!=0)
      require(cached.motors[0].preview_position==.05,"old phase must reproduce stale target");
    cached.ipc.motor_preview_count=phase;
    preview_final_motor_commands(cached.motors,cached.ipc,cached.kinematics,false,true,true);
    preview_final_motor_commands(fresh.motors,fresh.ipc,fresh.kinematics,false,true,true);
    for (std::size_t i=0;i<kJointCount;++i) {
      require(protected_policy_payload(cached.motors[i],false,true)==
              protected_policy_payload(fresh.motors[i],false,true),
              "coherent payload must use the newest action for every motor");
    }
    require(cached.motors[0].preview_position==.10,"new action must replace cached direct target");
    require(cached.motors[0].preview_kp==20. && cached.motors[0].preview_kd==1.,
            "commit must preserve gains");
  }
}
