#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/Float32.h"
#include "math.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class KeyBoardControl
{
public:
  KeyBoardControl();
  void Loop();

private:
  float min_scale_ = 3.0;
  float max_scale_ = 24.0;
  float max_angle = 360;
  float min_angel = -360;
  float step = 1.0;
  ros::NodeHandle nh_;
  float angular_, scale_;
  ros::Publisher angle_pub_;
  
};

KeyBoardControl::KeyBoardControl():
  angular_(0),
  scale_(3.0)
{
  angle_pub_ = nh_.advertise<std_msgs::Float32>("ros_talon/steering_angle", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_control");
  KeyBoardControl control_;

  signal(SIGINT,quit);

  control_.Loop();
  
  return(0);
}


void KeyBoardControl::Loop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the Wheel.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_INFO("LEFT");
        angular_ += -1;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_INFO("RIGHT");
        angular_ += 1;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_INFO("SpeedUp");
        scale_ += step;
        if (scale_ > max_scale_) scale_ = max_scale_;
        ROS_INFO("Scale = %f",scale_);
        break;
      case KEYCODE_D:
        ROS_INFO("SpeedDown");
        scale_ -= step;
        if (scale_ < min_scale_) scale_ = min_scale_;
        ROS_INFO("Scale = %f",scale_);
        break;
    }
   

    std_msgs::Float32 angle;
    
    angle.data = scale_*angular_;

    // adding angle limitation
    if (angle.data > max_angle) {
      angle.data = max_angle;
      angular_ -= 1;
    }
    else if (angle.data < min_angel) {
      angle.data = min_angel;
      angular_ += 1;
    }


    if (dirty == true) {
        
        angle_pub_.publish(angle);
        dirty = false;
    }
  }


  return;
}



