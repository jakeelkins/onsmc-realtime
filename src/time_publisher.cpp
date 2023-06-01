#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "onsmc_rt/msg/time.hpp"

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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TimePublisher : public rclcpp::Node{
public:

    bool go_flag = false;
    float t = 0.0f;  // actual t value we publish

    // nominal control period (s)
    float control_period = 0.01;

    // needed for our timer: ms duration cast
    std::chrono::duration<long double, std::milli> control_period_ms = control_period*1000ms;

    TimePublisher():Node("time_publisher"), count_(0){
        // subscribe to "go" topic (bool)
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "go", 10, std::bind(&TimePublisher::go_callback, this, std::placeholders::_1));

        // publish to time topic "t"
        publisher_ = this->create_publisher<onsmc_rt::msg::Time>("t", 10);

        timer_ = this->create_wall_timer(
        control_period_ms, std::bind(&TimePublisher::timer_callback, this));
    }

private:

    size_t count_;  // TODO: what is this used for ?

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<onsmc_rt::msg::Time>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;

    void timer_callback(){
        auto message = onsmc_rt::msg::Time();

        // timer is zero until it gets the "go" command.
        if (go_flag){
            t += control_period;
        }

        message.curr_t = t;
        //RCLCPP_INFO(this->get_logger(), "Publishing: %f", message.curr_t);
        publisher_->publish(message);
    }

    void go_callback(const std_msgs::msg::Bool & msg){
        // sets the flag to true
        if (msg.data){
            RCLCPP_INFO_ONCE(this->get_logger(), "Go flag received: experiment start.");
            go_flag = true;
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
    rclcpp::spin(std::make_shared<TimePublisher>());

  }
};

void LockMemory() {
  int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
  if (ret) {
    throw std::runtime_error{std::string("mlockall failed: ") + std::strerror(errno)};
  }
}

// ---------------------

int main(int argc, char * argv[])
{

  LockMemory();

  rclcpp::init(argc, argv);
  
  Thread rt_thread(80, SCHED_FIFO);

  rt_thread.Start();
  rt_thread.Join();

  rclcpp::shutdown();
  return 0;
}
