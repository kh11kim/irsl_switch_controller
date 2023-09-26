#include "core.h"
#include "loop_timer.h"
#include "thread_pool.h"
#include "key.h"

//interfaces
// 'control_variables', 'state', 'model', 'which controller uses which information'...

// robot: torque_ctrl_loop, idle_loop, pos_ctrl_loop...
// sensor: how to get sensor value in the loop
// controller how to calculate control input values using
inline double JointPlusOnlyMap(double theta){
  if(theta < 0){
    return theta + M_PI*2;
  }
  return theta;
}
inline double JointPlusMinusMap(double theta){
  if(theta >= M_PI){
    return theta - M_PI*2;
  }
  return theta;
}
#define DEGREE_TO_RADIAN (double)(M_PI/180)
#define RADIAN_TO_DEGREE (double)(180/M_PI)



struct Config{
  std::mutex m;
  bool is_running;
};
struct State{
  std::mutex m;
  bool is_refreshed_in_ctrl_loop;
  //current state
  Vector7d q;
  Vector7d dq;
  //desired state
  Vector7d q_d;
  Vector7d dq_d;
};

class PosCtrlVelMode{
private:
  double kp;
public:
  Vector7d error;

  PosCtrlVelMode():kp(2.){error.setZero();}
  Vector7d ControlLaw(const State &state){
    for (int i=0;i<7;i++){
      this->error[i] = state.q_d[i] - state.q[i];
    }
    return kp * this->error;
  }
};

enum ControlMode{
  IDLE = 0,
  JOINT_POSITION_CTRL_VEL_MODE,
};
typedef std::function<Vector7d(const State&)> ControlLaw;
typedef std::function<void()> ControlLoop;
typedef std::function<void()> ControlQuit;

struct KinovaClients{
  int num_port = 10000;
  int num_port_rt = 10001;
  std::string username = "admin";
  std::string userpw = "admin";
  Kinova::Api::RouterClient *router;
  Kinova::Api::RouterClient *router_real_time;
  Kinova::Api::TransportClientTcp *transport;
  Kinova::Api::TransportClientUdp *transport_real_time;
  Kinova::Api::Base::BaseClient *base;
  Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic;
  Kinova::Api::SessionManager *session_manager;
  Kinova::Api::SessionManager *session_manager_real_time;
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *actuator_config;
  
  // messages
  Kinova::Api::BaseCyclic::Feedback base_feedback;
  Kinova::Api::BaseCyclic::Command base_command;

  void Init(std::string ip){
    // Create API objects
    auto error_callback = [](Kinova::Api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };
    this->transport = new Kinova::Api::TransportClientTcp();
    this->router = new Kinova::Api::RouterClient(this->transport, error_callback);
    this->transport->connect(ip, this->num_port);

    std::cout << "Creating transport real time objects" << std::endl;
    this->transport_real_time = new Kinova::Api::TransportClientUdp();
    this->router_real_time = new Kinova::Api::RouterClient(this->transport_real_time, error_callback);
    transport_real_time->connect(ip, this->num_port_rt);

    // Set session data connection information
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username(this->username);
    create_session_info.set_password(this->userpw);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    this->session_manager = new Kinova::Api::SessionManager(this->router);
    session_manager->CreateSession(create_session_info);
    this->session_manager_real_time = new Kinova::Api::SessionManager(this->router_real_time);
    this->session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    this->base = new Kinova::Api::Base::BaseClient(this->router);
    this->base_cyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(this->router_real_time);
    this->actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(router);

    InitializePositionCommand();
  }

  void InitializePositionCommand(){
    // Initialize current states
    this->base_feedback = this->base_cyclic->RefreshFeedback();
    for (int i=0;i<7;i++){
      this->base_command.add_actuators()->set_position(this->base_feedback.actuators(i).position());
    }
  }
  
  void Finish(){
    // Close API session
    this->session_manager->CloseSession();
    this->session_manager_real_time->CloseSession();
    std::cout << "Session closed" <<std::endl;
    
    // Deactivate the router and cleanly disconnect from the transport object
    this->router->SetActivationStatus(false);
    this->transport->disconnect();
    this->router_real_time->SetActivationStatus(false);
    this->transport_real_time->disconnect();
    std::cout << "Router/transport deactivated" <<std::endl;

    // Destroy the API
    delete this->base;
    delete this->base_cyclic;
    delete this->actuator_config;
    delete this->session_manager;
    delete this->session_manager_real_time;
    delete this->router;
    delete this->router_real_time;
    delete this->transport;
    delete this->transport_real_time;
    std::cout << "Memory deallocated" <<std::endl;
  }

  void SetKinovaServoingMode(Kinova::Api::Base::ServoingMode servoing_mode){
    // Set the base in low-level servoing mode
    auto servoing_mode_msg = Kinova::Api::Base::ServoingModeInformation();
    servoing_mode_msg.set_servoing_mode(servoing_mode);
    this->base->SetServoingMode(servoing_mode_msg);
    std::cout << "Kinova: Change servoing mode" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  void SetKinovaControlMode(Kinova::Api::ActuatorConfig::ControlMode control_mode){
    this->base->ClearFaults();

    // Set all actuators in the corresponding control mode.
    std::cout << "Kinova: Change control mode" << std::endl;
    auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(control_mode);
    for (int i = 0; i < 7; i++){
      this->actuator_config->SetControlMode(control_mode_message, i+1);
    }
  }
};

// class Panda{
// protected:
//   std::string ip = "172.16.0.2";
//   franka::Robot client;
//   franka::RobotState panda_state;

// public:
//   CtrlVar ctrl_var;
//   State state;
  

//   Panda(): client(this->ip) {
//     ;
//   }
//   void UpdateState(const franka::RobotState &panda_state);
//   void IdleLoop();
// };

// void Panda::UpdateState(const franka::RobotState &panda_state){
//   std::lock_guard<std::mutex> guard(this->state.m);
//   this->panda_state = panda_state;
//   ArrayToEigen(panda_state.q, this->state.q);
//   //std::cout << this->state.q.transpose() << std::endl;
// }

// void Panda::IdleLoop(){
//   std::function<bool(const franka::RobotState&)> loop_fn_panda; 

//   loop_fn_panda = [&](const franka::RobotState& panda_state){
//     UpdateState(panda_state);
//     return this->ctrl_var.is_running; //if stop_ctrl is true, break
//   };
//   this->client.read(loop_fn_panda);
// }

class Gen3{
protected:
  std::string ip = "172.16.0.8";
  KinovaClients client;
  //KinovaMsgs msgs;
  LoopTimer vel_loop_timer;
  ThreadPool tp;

  ControlLaw ctrl_law;
  ControlLoop ctrl_loop;
  ControlQuit ctrl_quit;

public:
  Config ctrl_var;
  State state;
  
  //controller
  PosCtrlVelMode pos_ctrl;
  
  Gen3();
  ~Gen3();
  
  void RunCtrl(ControlMode mode);
  void StopCtrl();
  // void StopThreadPool();
  void UpdateState();
  void ApplyVelocityInput(const Vector7d &velocity);

  void IdleLoop();
  void VelocityModeInit();
  void VelocityModeLoop(ControlLaw ctrl_loop_fn);
  void VelocityModeQuit();
};

Gen3::Gen3()
: vel_loop_timer(1000) {
  this->client.Init(this->ip);
}

Gen3::~Gen3(){
  this->tp.StopThreadPool();
  this->client.Finish();
}

void Gen3::RunCtrl(ControlMode control_mode){
  using namespace std::placeholders;  

  switch(control_mode){
    case IDLE:
      std::cout << "IDLE Mode" << std::endl;
      this->client.SetKinovaServoingMode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);

      this->ctrl_loop = std::bind(&Gen3::IdleLoop, this);
      this->ctrl_quit = std::bind(&Gen3::VelocityModeQuit, this);
    break;
    case JOINT_POSITION_CTRL_VEL_MODE:
      std::cout << "Position Control w/ Velocity Mode" << std::endl;
      this->VelocityModeInit();
      this->ctrl_law = std::bind(&PosCtrlVelMode::ControlLaw, this->pos_ctrl, _1);
      this->ctrl_loop = std::bind(&Gen3::VelocityModeLoop, this, ctrl_law);
      this->ctrl_quit = std::bind(&Gen3::VelocityModeQuit, this);
    break;
  }
  this->tp.EnqueueJob([this](){this->ctrl_loop();});
}

void Gen3::StopCtrl(){
  {
    std::lock_guard<std::mutex> guard(this->ctrl_var.m);
    ctrl_var.is_running = false;
  }
  this->ctrl_quit();
}

void Gen3::UpdateState(){
  std::lock_guard<std::mutex> guard(this->state.m);
  if (!this->state.is_refreshed_in_ctrl_loop){
    this->client.base_feedback = this->client.base_cyclic->RefreshFeedback();
  }
  for(int i=0; i<7;i++){
    float theta = this->client.base_feedback.actuators(i).position();
    this->state.q[i] = JointPlusMinusMap((double)theta * DEGREE_TO_RADIAN);
    this->state.dq[i] = (double)this->client.base_feedback.actuators(i).velocity() * DEGREE_TO_RADIAN; // motor velocity
    // tau_J_raw(i) = -(double)base_feedback.actuators(i).torque(); // joint torque sensor
  }
  this->state.is_refreshed_in_ctrl_loop = false;
}
void Gen3::ApplyVelocityInput(const Vector7d &velocity){
  try{
    for(int i=0;i<7;i++){
      this->client.base_command.mutable_actuators(i)->set_velocity(velocity[i] * RADIAN_TO_DEGREE);
    }
    this->client.base_feedback = this->client.base_cyclic->Refresh(this->client.base_command);
    this->state.is_refreshed_in_ctrl_loop = true;
  }
  catch (Kinova::Api::KDetailedException& ex){
      std::cout << "Kortex error: " << ex.what() << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
      std::cout << "Runtime error: " << ex2.what() << std::endl;
  }
}

void Gen3::IdleLoop(){
  std::cout << " Idle Loop \n";
  while (this->ctrl_var.is_running){
    vel_loop_timer.Start();
    this->UpdateState();
    vel_loop_timer.Sleep();
  }
}

void Gen3::VelocityModeInit(){
  this->client.SetKinovaServoingMode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  this->client.SetKinovaControlMode(Kinova::Api::ActuatorConfig::ControlMode::VELOCITY);
  this->UpdateState();
  state.q_d = state.q;
}

void Gen3::VelocityModeLoop(ControlLaw ctrl_loop_fn){
  Vector7d velocity;
  while (this->ctrl_var.is_running){
    vel_loop_timer.Start();
    this->UpdateState();
    velocity = ctrl_loop_fn(this->state);
    this->ApplyVelocityInput(velocity);
    vel_loop_timer.Sleep();
  }
}

void Gen3::VelocityModeQuit(){
  // quit
  this->client.InitializePositionCommand();
  this->client.SetKinovaControlMode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
  this->client.SetKinovaServoingMode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  std::cout << "quit velocity mode" << std::endl;
}


bool keyboard_interface(Gen3 &robot){
  bool is_running = true;

  if (kbhit()){
    int key_pressed = getchar();
    switch(key_pressed){
      case 'q':
        robot.StopCtrl();
        is_running = false;
      break;
      default:
        is_running = true;
      break;
    }
  }
  return is_running;
}


int main(int, char**){
  init_keyboard();

  //Panda panda;
  Gen3 gen3;
  
  std::this_thread::sleep_for(std::chrono::seconds(3));
  gen3.RunCtrl(JOINT_POSITION_CTRL_VEL_MODE); //run joint position control  JOINT_POSITION_CTRL_VEL_MODE
  std::this_thread::sleep_for(std::chrono::seconds(1));

  LoopTimer timer(10);

  bool is_running = true;
  while(is_running){
    timer.Start();
    
    //print
    {
      std::lock_guard<std::mutex> guard(gen3.state.m);
      std::cout << "q  : " << gen3.state.q.transpose() << std::endl;
      std::cout << "q_d: " << gen3.state.q_d.transpose() << std::endl;
      std::cout << "err: " << gen3.pos_ctrl.error.transpose() << std::endl;
    }
    timer.Sleep();
    
    is_running = keyboard_interface(gen3);
  }
  
  close_keyboard();
  return 0;
}
