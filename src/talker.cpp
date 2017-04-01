#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>   //_getch
#include <termios.h>

//pobieranie znaku
char getch(){
    
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
 //   printf("%c\n",buf);
    return buf;
 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

	char c;
    if(c=getch()){
        msg.linear.x=0.0;
		msg.linear.y=0.0;
		msg.linear.z=0.0;
		msg.angular.x=0.0;
		msg.angular.y=0.0;
		msg.angular.z=0.0;
	if(c=='w')
    {
		msg.linear.x=2.0;
		msg.linear.y=0.0;
		msg.linear.z=0.0;
		msg.angular.x=0.0;
		msg.angular.y=0.0;
		msg.angular.z=0.0;
        
	}
	if(c=='s')
    {
		msg.linear.x=-2.0;
		msg.linear.y=0.0;
		msg.linear.z=0.0;
		msg.angular.x=0.0;
		msg.angular.y=0.0;
		msg.angular.z=0.0;
        
	}
	if(c=='a')
    {
		msg.linear.x=0.0;
		msg.linear.y=0.0;
		msg.linear.z=0.0;
		msg.angular.x=0.0;
		msg.angular.y=0.0;
		msg.angular.z=2.0;
        
	}
	if(c=='d')
    {
		msg.linear.x=0.0;
		msg.linear.y=0.0;
		msg.linear.z=0.0;
		msg.angular.x=0.0;
		msg.angular.y=0.0;
		msg.angular.z=-2.0;
       
	}	
    
	}    
    
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
