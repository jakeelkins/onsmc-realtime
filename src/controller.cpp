#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <cstdio>
#include <vector>
#include <random>
#include <cmath>
// #include <string>
// #include <fstream>
// #include <vector>
// #include <utility> // std::pair

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "onsmc_rt/msg/desired_trajectory.hpp"
#include "onsmc_rt/msg/control.hpp"
#include "onsmc_rt/msg/state.hpp"

#include "onsmc_rt/onsmc.h"

//***********************************************
// START: This part is for real-time setup
#include <pthread.h>
#include <sys/mman.h>  // necessary for mlockall

#include <cstring>
#include <stdexcept>
#include <string>
// END: This part is for real-time setup
//***********************************************

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class Controller: public rclcpp::Node {
public:

  // nominal control frequency (s) CURRENT: ensure same as time publisher
  float control_period = 0.01;

  // needed for our timer: ms duration cast
  chrono::duration<long double, std::milli> control_period_ms = control_period*1000ms;


  Controller()
  :Node("controller"),
  onsmc(input_dim, output_dim, control_period)
  {

    // populate vector ICs in constructor:
    vector<float> _y (
        output_dim,
        0.0f
    );

    vector<float> _y_dot (
        output_dim,
        0.0f
    );

    vector<float> _yd (
        output_dim,
        0.0f
    );

    vector<float> _yd_dot (
        output_dim,
        0.0f
    );

    vector<float> _yd_ddot (
        output_dim,
        0.0f
    );

    // variables needed
    vector<float> _u (
        output_dim,
        0.0f
    );

    y = _y;
    y_dot = _y_dot;
    
    yd = _yd;
    yd_dot = _yd_dot;
    yd_ddot = _yd_ddot;

    u = _u;


    // need to subscribe to "go" flag so I can plot the learning from the very start
    subscription_go = this->create_subscription<std_msgs::msg::Bool>(
            "go", 10, std::bind(&Controller::go_callback, this, std::placeholders::_1));

    // subscribe to current state x as input
    subscription_x = this->create_subscription<onsmc_rt::msg::State>(
      "state", 10, std::bind(&Controller::state_receive_callback, this, _1));

    // subscribe to topic desired_trajectory as input
    subscription_xd = this->create_subscription<onsmc_rt::msg::DesiredTrajectory>(
      "desired_trajectory", 10, std::bind(&Controller::traj_receive_callback, this, _1));

    // publish to control topic
    publisher_ = this->create_publisher<onsmc_rt::msg::Control>("control", 10);

    // wall timer for publishing control
    timer_ = this->create_wall_timer(
        control_period_ms, std::bind(&Controller::control_callback, this));

  }

private:

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_go;
  rclcpp::Subscription<onsmc_rt::msg::State>::SharedPtr subscription_x;
  rclcpp::Subscription<onsmc_rt::msg::DesiredTrajectory>::SharedPtr subscription_xd;
  rclcpp::Publisher<onsmc_rt::msg::Control>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool go_flag = false;

  unsigned int input_dim = 7;
  unsigned int output_dim = 1;

  // ONSMC controller
  ONSMC onsmc;

  // --  state ICs --
  // match these to the desireds so u is 0 before the experiment runs
  float q = 0.0f;
  float q_dot = 0.0f;
  float I = 0.0f;

  // -- desired trajectory ICs --
  float qd = 0.0f;
  float qd_dot = 0.0f;
  float qd_ddot = 0.0f;

  // use vectors
  vector<float> y;
  vector<float> y_dot;

  vector<float> yd;
  vector<float> yd_dot;
  vector<float> yd_ddot;

  vector<float> u;

  void go_callback(const std_msgs::msg::Bool &msg){
        // sets the flag to true
        if (msg.data){
            //RCLCPP_INFO_ONCE(this->get_logger(), "Go flag received: experiment start.");
            go_flag = true;
        }
  }


  void state_receive_callback(const onsmc_rt::msg::State &state_msg){

    // -- record current state inputs --
    q = state_msg.q;
    q_dot = state_msg.q_dot;
    I = state_msg.curr_i;

    // put the state values into the vectors
    y[0] = q;
    y_dot[0] = q_dot; 

  }

  void traj_receive_callback(const onsmc_rt::msg::DesiredTrajectory &desired_traj_msg){

    // -- record desired trajectory inputs --
    qd = desired_traj_msg.qd;
    qd_dot = desired_traj_msg.qd_dot;
    qd_ddot = desired_traj_msg.qd_ddot;

    // put the desireds in the vector
    yd[0] = qd;
    yd_dot[0] = qd_dot;
    yd_ddot[0] = qd_ddot;

  }

  // THE MEAT: controller.
  void control_callback(){

    // out msg:
    auto control_msg = onsmc_rt::msg::Control();

    //RCLCPP_INFO(this->get_logger(), "curr q: %f  curr q_dot: %f  curr qd: %f", q, q_dot, qd);
    //RCLCPP_INFO_ONCE(this->get_logger(), "input_dim: %i  hidden_dim: %i  output_dim: %i", onsmc.input_dim, onsmc.hidden_dim, onsmc.output_dim);
    
    u[0] = 0.0f;

    if (go_flag){
      RCLCPP_INFO_ONCE(this->get_logger(), "Controller received go flag");
      // ---- control calcs ----
      // get control: puts it into u vector.
      onsmc.get_control(u.data(), y.data(), y_dot.data(),
                        yd.data(), yd_dot.data(), yd_ddot.data());

      // scale
      //u[0] = 0.12*u[0];

      u[0] = 0.105f*u[0];
      //u[0] = 0.02f*u[0];

      // deadzone inverse
      //float dz = 0.335f;
      float dz = 0.2f;
      //float dz = 0.0f;

      if (u[0] > 0.0f){
        u[0] = u[0] + dz;
      } else if (u[0] < 0.0f){
        u[0] = u[0] - dz;
      }

      

      // clip
      //float clip = 3.0f;
      float clip = 1.335f;

      if (u[0] > clip){
        u[0] = clip;
      } else if (u[0] < -clip){
        u[0] = -clip;
      }

      // ONLY FOR EXPERIMENT: get norms and publish
      // frobenius:
      float M_F = 0.0f;

      for (int i = 0; i<onsmc.output_dim; i++){
        M_F += onsmc.M_hat[i]*onsmc.M_hat[i];
      }

      M_F = sqrt(M_F);

      control_msg.mf = M_F;

      float W_F = 0.0f;

      for (int i = 0; i<(onsmc.output_dim*onsmc.hidden_dim); i++){
        W_F += onsmc.NN.W[i]*onsmc.NN.W[i];
      }

      W_F = sqrt(W_F);

      control_msg.wf = W_F;

      float V_F = 0.0f;

      for (int i = 0; i<(onsmc.input_dim*onsmc.hidden_dim); i++){
        V_F += onsmc.NN.V[i]*onsmc.NN.V[i];
      }

      V_F = sqrt(V_F);

      control_msg.vf = V_F;

      control_msg.s = onsmc.s[0];
    }

    // ------------------------

    control_msg.u = u[0];

    publisher_->publish(control_msg);

    if (qd < -0.9){
      RCLCPP_INFO_ONCE(this->get_logger(), "W[0]: %f", onsmc.NN.W[0]);
      RCLCPP_INFO_ONCE(this->get_logger(), "M_hat[0]: %f", onsmc.M_hat[0]);
    }
  }
};


class Thread {
  int priority_;
  int policy_;

  pthread_t thread_;

  static void* RunThread(void* data) {
    Thread* thread = static_cast<Thread*>(data);
    thread->Run();
    return NULL;
  }

 public:
  Thread(int priority, int policy)
      : priority_(priority), policy_(policy) {}
  virtual ~Thread() = default;

  // Delete the copy constructor; default the move constructor...

  void Start() {
    pthread_attr_t attr;

    // Initialize the pthread attribute
    int ret = pthread_attr_init(&attr);
    if (ret) {
      throw std::runtime_error(std::string("error in pthread_attr_init: ") + std::strerror(ret));
    }

    // Set the scheduler policy
    ret = pthread_attr_setschedpolicy(&attr, policy_);
    if (ret) {
      throw std::runtime_error(std::string("error in pthread_attr_setschedpolicy: ") + std::strerror(ret));
    }

    // Set the scheduler priority
    struct sched_param param;
    param.sched_priority = priority_;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
      throw std::runtime_error(std::string("error in pthread_attr_setschedparam: ") + std::strerror(ret));
    }

    // Make sure threads created using the thread_attr_ takes the value from the attribute instead of inherit from the parent thread.
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
      throw std::runtime_error(std::string("error in pthread_attr_setinheritsched: ") + std::strerror(ret));
    }

    ret = pthread_create(&thread_, &attr, &Thread::RunThread, this);
    if (ret) {
      throw std::runtime_error(std::string("error in pthread_create: ") + std::strerror(ret));
    }
  }

  int Join() {
    return pthread_join(thread_, NULL);
  }

  virtual void Run() noexcept {
    // Code here will run as RT
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Controller>();
    rclcpp::spin(node);

  }
};

void LockMemory() {
  int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
  if (ret) {
    throw std::runtime_error{std::string("mlockall failed: ") + std::strerror(errno)};
  }
}


int main(int argc, char * argv[]){

  LockMemory();

  rclcpp::init(argc, argv);
  
  Thread rt_thread(80, SCHED_FIFO);
  rt_thread.Start();
  rt_thread.Join();
  
  rclcpp::shutdown();
  return 0;
}