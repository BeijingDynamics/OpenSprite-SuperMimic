// Exercise the production feedback decoder using Unix sockets, never a CAN interface.
void test_feedback_observation() {
  auto attempt = [](int status, unsigned position, unsigned velocity, unsigned torque,
                    int temperature, bool observe) {
    int peer[2];
    require(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, peer)==0, "feedback socketpair");
    Fixture f;
    auto& motor = f.motors[0];
    motor.name="right_knee_motor";motor.can_id=4;motor.poll_rate_hz=50;motor.rated_torque=14;
    motor.feedback_torque_cap=28.-56./8190.;motor.feedback_torque_observe_only=observe;
    motor.mos_temperature_limit=120;motor.rotor_temperature_limit=100;
    motor.filtered_mos_temperature_limit=90;motor.filtered_rotor_temperature_limit=80;
    motor.temperature_filter_time_constant_s=2;
    std::vector<Socket> sockets(1);sockets[0].interface="unit";sockets[0].fd=peer[0];
    canfd_frame frame{};frame.can_id=20;frame.len=8;frame.flags=CANFD_BRS;
    frame.data[0]=(status<<4)|4;frame.data[1]=position>>8;frame.data[2]=position&255;
    frame.data[3]=velocity>>4;frame.data[4]=((velocity&15)<<4)|(torque>>8);
    frame.data[5]=torque&255;frame.data[6]=temperature;frame.data[7]=30;
    require(write(peer[1],&frame,CANFD_MTU)==CANFD_MTU,"feedback test frame");
    std::string error;
    try {drain_policy_feedback(sockets,f.motors,{{{"unit",20},0}},1,true);}
    catch(const std::runtime_error& e){error=e.what();}
    close(peer[0]);close(peer[1]);
    if(error.empty() && (torque==0 || torque==4095)) {
      require(motor.feedback_torque_excursions==1 && motor.feedback_torque_full_scale_samples==1,
              "full-scale feedback must be recorded");
      require(std::abs(motor.first_torque_excursion_nm)==28.,"first excursion value retained");
    }
    return error;
  };
  for(unsigned torque : {0u,4095u}) {
    require(attempt(1,32768,2048,torque,30,true).empty(),"instantaneous full-scale torque is observational");
    require(attempt(1,32768,2048,torque,30,false).find("reason=feedback_torque")!=std::string::npos,
            "other profiles retain instantaneous torque stop");
  }
  require(attempt(8,32768,2048,4095,30,true).find("identity/status")!=std::string::npos,
          "motor fault status must still stop");
  require(attempt(1,0,2048,4095,30,true).find("reason=position")!=std::string::npos,
          "position guard must survive concurrent torque observation");
  require(attempt(1,32768,4095,4095,30,true).find("reason=velocity")!=std::string::npos,
          "velocity guard must survive concurrent torque observation");
  require(attempt(1,32768,2048,4095,121,true).find("temperature")!=std::string::npos,
          "temperature guard must remain active");
}
