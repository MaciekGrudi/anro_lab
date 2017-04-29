#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"


using namespace std;
void zbudujMacierz();
void callback(const sensor_msgs::JointState & msg);
void CalculateQuaternion();

double t1,t3,t4,a3=0.4, a4=0.2,d2=0.2;
double T[4][4];
double Quat[4];

double init_t1=0;
double init_t3=-M_PI/4;
double init_t4=M_PI/4;

int main(int argc, char **argv)
{	
    //inicjacja ros
	ros::init(argc, argv, "nonkdl");
	ros::NodeHandle n;
    //utworzenie pub i sub
	ros::Publisher nonkdl_pub=n.advertise<geometry_msgs::PoseStamped>("posestamped_nonkdl",1);
	ros::Subscriber nonkdl_sub=n.subscribe("joint_states",1000,callback);	

	ros::Rate loop_rate(100);
	geometry_msgs::PoseStamped msg;
		
	while(ros::ok())
	{
	      
       	
		//wypełenienie wiadomości treścią
		msg.header.frame_id="base_link";
		msg.pose.orientation.x=Quat[0];
		msg.pose.orientation.y=Quat[1];
		msg.pose.orientation.z=Quat[2];
		msg.pose.orientation.w=Quat[3];
		msg.pose.position.x=T[0][3];
		msg.pose.position.y=T[1][3];
		msg.pose.position.z=T[2][3];
	    
	    //publish
		nonkdl_pub.publish(msg);
	
		ros::spinOnce();
	
		loop_rate.sleep();
	}

	return 0;
}

void zbudujMacierz()
{
	T[0][0]=cos(t1)*cos(t3)*cos(t4) - cos(t1)*sin(t3)*sin(t4);
	T[0][1]=-cos(t1)*cos(t3)*sin(t4) - cos(t1)*cos(t4)*sin(t3);
	T[0][2]=-sin(t1);
	T[0][3]=(2*cos(t1)*cos(t3))/5 - (cos(t1)*sin(t3)*sin(t4))/5 + (cos(t1)*cos(t3)*cos(t4))/5;
	T[1][0]=cos(t3)*cos(t4)*sin(t1) - sin(t1)*sin(t3)*sin(t4);
	T[1][1]=-cos(t3)*sin(t1)*sin(t4) - cos(t4)*sin(t1)*sin(t3);
	T[1][2]=cos(t1);
	T[1][3]=(2*cos(t3)*sin(t1))/5 - (sin(t1)*sin(t3)*sin(t4))/5 + (cos(t3)*cos(t4)*sin(t1))/5;
	T[2][0]=-cos(t3)*sin(t4) - cos(t4)*sin(t3);
	T[2][1]=sin(t3)*sin(t4) - cos(t3)*cos(t4);
	T[2][2]=0;
	T[2][3]=0.2-(cos(t3)*sin(t4))/5 - (cos(t4)*sin(t3))/5 - (2*sin(t3))/5;
	T[3][0]=0;
	T[3][1]=0;
	T[3][2]=0;
	T[3][3]=1;
}

void callback(const sensor_msgs::JointState & msg)
{
	t1=msg.position[0];
	t3=msg.position[1];
	t4=msg.position[2];
	
	t1+=init_t1;
	t3+=init_t3;
	t4+=init_t4;
	
	zbudujMacierz();
	CalculateQuaternion();
	
} 

void CalculateQuaternion()
{
  float trace = T[0][0] + T[1][1] + T[2][2]; 
  if( trace > 0 ) {
    float s = 0.5f / sqrtf(trace+ 1.0f);
    Quat[3] = 0.25f / s;
    Quat[0] = ( T[2][1] - T[1][2] ) * s;
    Quat[1] = ( T[0][2] - T[2][0] ) * s;
    Quat[2] = ( T[1][0] - T[0][1] ) * s;
  } else {
    if ( T[0][0] > T[1][1] && T[0][0] > T[2][2] ) {
      float s = 2.0f * sqrtf( 1.0f + T[0][0] - T[1][1] - T[2][2]);
      Quat[3] = (T[2][1] - T[1][2] ) / s;
      Quat[0] = 0.25f * s;
      Quat[1] = (T[0][1] + T[1][0] ) / s;
      Quat[2] = (T[0][2] + T[2][0] ) / s;
    } else if (T[1][1] > T[2][2]) {
      float s = 2.0f * sqrtf( 1.0f + T[1][1] - T[0][0] - T[2][2]);
      Quat[4] = (T[0][2] - T[2][0] ) / s;
      Quat[0] = (T[0][1] + T[1][0] ) / s;
      Quat[1] = 0.25f * s;
      Quat[2] = (T[1][2] + T[2][1] ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + T[2][2] - T[0][0] - T[1][1] );
      Quat[3] = (T[1][0] - T[0][1] ) / s;
      Quat[0] = (T[0][2] + T[2][0] ) / s;
      Quat[1] = (T[1][2] + T[2][1] ) / s;
      Quat[2] = 0.25f * s;
    }
  }
  
}




