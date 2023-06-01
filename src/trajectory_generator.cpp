#include <functional>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "onsmc_rt/msg/time.hpp"
#include "onsmc_rt/msg/desired_trajectory.hpp"

//***********************************************
// START: This part is for real-time setup
#include <pthread.h>
#include <sys/mman.h>  // necessary for mlockall

#include <cstring>
#include <stdexcept>
#include <string>
// END: This part is for real-time setup
//***********************************************

using std::placeholders::_1;

class TrajectoryGenerator : public rclcpp::Node {
public:

  TrajectoryGenerator():Node("trajectory_generator"){

    // subscribe to topic t as input
    subscription_ = this->create_subscription<onsmc_rt::msg::Time>(
      "t", 10, std::bind(&TrajectoryGenerator::t_received_callback, this, _1));

    // publish to desired_trajectory topic
    publisher_ = this->create_publisher<onsmc_rt::msg::DesiredTrajectory>("desired_trajectory", 10);

  }

private:

  rclcpp::Subscription<onsmc_rt::msg::Time>::SharedPtr subscription_;
  rclcpp::Publisher<onsmc_rt::msg::DesiredTrajectory>::SharedPtr publisher_;

  void t_received_callback(const onsmc_rt::msg::Time &msg){

    auto desired_traj_msg = onsmc_rt::msg::DesiredTrajectory();
    float t = msg.curr_t;

    //RCLCPP_INFO(this->get_logger(), "Current t: %f s", msg.curr_t);

    // -- desired trajectory calcs --
    // float qd = 1 - cos(t);
    // float qd_dot = sin(t);
    // float qd_ddot = cos(t);

    float qd = sin(t/10);
    float qd_dot = cos(t/10)/10;
    float qd_ddot = -sin(t/10)/100;

    // float qd = 1;
    // float qd_dot = 0.0f;
    // float qd_ddot = 0.0f;

    // float qd = 0.0f;
    // float qd_dot = 0.0f;
    // float qd_ddot = 0.0f;

    // if (t > 2.0f){
    //   //qd = 30*(3.14159265/180);
    //   qd = 0.3;
    // }
    
    // if (t > 2.2f) {
    //   qd = 0.0f;
    // }

    // if (t > 3.0f) {
    //   qd = -0.3f;
    // }

    // if (t > 3.2f) {
    //   qd = 0.0f;
    // }

    desired_traj_msg.qd = qd;
    desired_traj_msg.qd_dot = qd_dot;
    desired_traj_msg.qd_ddot = qd_ddot;

    //RCLCPP_INFO(this->get_logger(), "Publishing desired trajectory...");

    publisher_->publish(desired_traj_msg);

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
    std::shared_ptr<rclcpp::Node> node = std::make_shared<TrajectoryGenerator>();
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
