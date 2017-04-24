#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_datatypes.h>

using namespace std;
void zbudujMacierz();
void callback(const sensor_msgs::JointState & msg);

double t1,t2,t3,a2=0.4,d2=0.2;
double T[4][4];

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "nonkdl");

	ros::NodeHandle n;

	ros::Publisher nonkdl_pub=n.advertise<geometry_msgs::PoseStamped>("posestamped_nonkdl",1000); //zmienic nazwe
	ros::Subscriber nonkdl_sub=n.subscribe("joint_states",1000,callback);	

	ros::Rate loop_rate(1);

	tf::Quaternion q;
	geometry_msgs::PoseStamped msg;
	double alpha,beta,gamma;
	
	while(ros::ok())
	{
		zbudujMacierz();
	
		gamma=-atan2(T[0][1],T[1][1]);
		alpha=0;
		beta=-M_PI/2.0;
		q.setRPY(alpha,beta,gamma);
		msg.header.frame_id="base_link";
		msg.pose.orientation.x=0;
		msg.pose.orientation.y=0;
		msg.pose.orientation.z=1;		
		msg.pose.orientation.w=0;

		msg.pose.position.x=T[3][0];
		msg.pose.position.y=T[3][1];
		msg.pose.position.z=T[3][2];
	
		nonkdl_pub.publish(msg);
	
		ros::spin();
	
		loop_rate.sleep();
	}

	return 0;
}

void zbudujMacierz()
{
	T[0][0]=cos(t1)*cos(t2)*cos(t3) - cos(t1)*sin(t2)*sin(t3);
	T[0][1]=-cos(t1)*cos(t2)*sin(t3) - cos(t1)*cos(t3)*sin(t2);
	T[0][2]=-sin(t1);
	T[0][3]=a2*cos(t1)*cos(t2) - d2*sin(t1);
	T[1][0]=cos(t2)*cos(t3)*sin(t1) - sin(t1)*sin(t2)*sin(t3);
	T[1][1]=-cos(t2)*sin(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2);
	T[1][2]=cos(t1);
	T[1][3]=d2*cos(t1) + a2*cos(t2)*sin(t1);
	T[2][0]=-cos(t2)*sin(t3) - cos(t3)*sin(t2);
	T[2][1]= sin(t2)*sin(t3) - cos(t2)*cos(t3);
	T[2][2]=0;
	T[2][3]=-a2*sin(t2);
	T[3][0]=0;
	T[3][1]=0;
	T[3][2]=0;
	T[3][3]=1;
}

void callback(const sensor_msgs::JointState & msg)
{
	t1 = msg.position[0];
	t2=msg.position[1];
	t3=msg.position[2];
} 

