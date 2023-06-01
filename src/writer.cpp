#include <functional>
#include <memory>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility> // std::pair

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "onsmc_rt/msg/time.hpp"
#include "onsmc_rt/msg/desired_trajectory.hpp"
#include "onsmc_rt/msg/state.hpp"
#include "onsmc_rt/msg/control.hpp"

using std::placeholders::_1;
using namespace std;

// the file we will save to.
ofstream myFile;

class Writer : public rclcpp::Node {
public:

    Writer():Node("writer"){

        // subscribe to "go" topic (bool)
        subscription_go = this->create_subscription<std_msgs::msg::Bool>(
            "go", 10, std::bind(&Writer::go_callback, this, std::placeholders::_1));

        // subscribe to topic t as input
        t_subscription = this->create_subscription<onsmc_rt::msg::Time>(
            "t", 10, std::bind(&Writer::t_received_callback, this, _1));

        // subscribe to desired trajectory
        subscription_xd = this->create_subscription<onsmc_rt::msg::DesiredTrajectory>(
            "desired_trajectory", 10, std::bind(&Writer::traj_received_callback, this, _1));

        subscription_x = this->create_subscription<onsmc_rt::msg::State>(
            "state", 10, std::bind(&Writer::state_received_callback, this, _1));

        // subscribe to control topic
        subscription_u = this->create_subscription<onsmc_rt::msg::Control>(
            "control", 10, std::bind(&Writer::control_received_callback, this, _1));

    }

private:

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_go;
  rclcpp::Subscription<onsmc_rt::msg::Time>::SharedPtr t_subscription;
  rclcpp::Subscription<onsmc_rt::msg::DesiredTrajectory>::SharedPtr subscription_xd;
  rclcpp::Subscription<onsmc_rt::msg::State>::SharedPtr subscription_x;
  rclcpp::Subscription<onsmc_rt::msg::Control>::SharedPtr subscription_u;

  bool go_flag = false;
  // current values for record callback
  float t;

  float qd;
  float qd_dot;
  float qd_ddot;

  float q;
  float q_dot;

  float u;

  float I;

  // str buffers to help
  char curr_buffer[200];
  //char save_format_str[] = "%f,%f,%f,%f,%f\n";

  void go_callback(const std_msgs::msg::Bool &msg){
        // sets the flag to true
        if (msg.data){
            //RCLCPP_INFO_ONCE(this->get_logger(), "Go flag received: experiment start.");
            go_flag = true;
            // open her up!
            myFile.open("/home/jake/ros2_ws/src/onsmc_rt/data/sim_out.csv");
            // put the headers
            myFile << "t,u,xd,x,I\n";
        }
  }

  void t_received_callback(const onsmc_rt::msg::Time &msg){

    t = msg.curr_t;

    //RCLCPP_INFO(this->get_logger(), "Current t: %f s", msg.curr_t);

  }

  void traj_received_callback(const onsmc_rt::msg::DesiredTrajectory &desired_traj_msg){

    qd = desired_traj_msg.qd;
    qd_dot = desired_traj_msg.qd_dot;
    qd_ddot = desired_traj_msg.qd_ddot;

  }

  void state_received_callback(const onsmc_rt::msg::State &state_msg){

    // -- record current state inputs --
    q = state_msg.q;
    q_dot = state_msg.q_dot;
    I = state_msg.curr_i;

  }

  void control_received_callback(const onsmc_rt::msg::Control &control_msg){

    u = control_msg.u;

    if (go_flag){
        // record when we get control
        sprintf(curr_buffer, "%f,%f,%f,%f,%f\n", t, u, qd, q, I);
        // write to file
        myFile << curr_buffer;

    }

  }
  
};


int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<Writer>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    // close her up
    myFile.close();
    return 0;
}
