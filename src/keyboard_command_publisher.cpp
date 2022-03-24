#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_A 0x61
#define KEYCODE_Z 0x7A
#define KEYCODE_S 0x73
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class Teleop
{
  public:
    Teleop();
    void keyLoop();
  private:
    ros::NodeHandle nh_;
    std::string input;
    ros::Publisher keyboard_pub_;
};

Teleop::Teleop():
  input("stop")
{
  keyboard_pub_ = nh_.advertise<std_msgs::String>("/qcar/keyboard_command", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop teleop;
  signal(SIGINT,quit);
  teleop.keyLoop();
  return(0);
}
void Teleop::keyLoop()
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
  puts("Use arrow keys to move the robot.");
  puts("Press the space bar to stop the robot.");
  puts("Press q to stop the program");
  //puts("a/z - Increase/decrease linear velocity");
  //puts("s/x - Increase/decrease angular velocity");
  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
	  {
  	  perror("read():");
  	  exit(-1);
	  }
    input = "stop";
    char printable[100];
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
	  {
    	case KEYCODE_L:
    	  ROS_DEBUG("LEFT");
        input = "left";
    	  dirty = true;
    	  break;
    	case KEYCODE_R:
    	  ROS_DEBUG("RIGHT");
        input = "right";
    	  dirty = true;
    	  break;
    	case KEYCODE_U:
    	  ROS_DEBUG("UP");
        input = "up";
    	  dirty = true;
    	  break;
    	case KEYCODE_D:
    	  ROS_DEBUG("DOWN");
        input = "down";
    	  dirty = true;
    	  break;
    	case KEYCODE_SPACE:
    	  ROS_DEBUG("STOP");
        input = "stop";
    	  dirty = true;
    	  break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        ROS_INFO_STREAM("You quit the teleop successfully");
        return;
        break;
  	}
    std_msgs::String keyboard_input;
    if(dirty ==true)
  	{
      keyboard_input.data = input;
  	  keyboard_pub_.publish(keyboard_input);
  	  dirty=false;
  	}
  }
  return;
}
