#include <kdl/chain.hpp>
#include <iostream>

using namespace std;
using namespace KDL;

int main()
{
	double t1,t2,t3,a2;
	
	cout<<"Podaj wartość theta_1:";
	cin>>t1;
	while(!cin.good()){
		cin.clear();
		cin.ignore(1000,'\n');
		cout<<"Podaj wartość theta_1:";
		cin>>t1;
	}
	
	do {
		cout<<"Podaj wartość theta_2:";
		cin.clear();
		cin.ignore(1000,'\n');
		cin>>t2;
	}
	while(!cin.good());

	do {
		cout<<"Podaj wartość theta_3:";
		cin.clear();
		cin.ignore(1000,'\n');
		cin>>t3;
	}
	while(!cin.good());

	do {
		cout<<"Podaj wartość a_2:";
		cin.clear();
		cin.ignore(1000,'\n');
		cin>>a2;
	}
	while(!cin.good());
	
	Chain chain;
	
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,0.1,t1))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,-M_PI/2.0,0,t2))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a2,0,0,t3))));	
	
	for(int i=0;i<3;i++)
	{
		double r,p,y;
		double pos[3];
		for(int j=0;j<3;j++) {
			pos[j]=chain.getSegment(i).getFrameToTip().p.data[j];
		}

		chain.getSegment(i).getFrameToTip().M.GetRPY(r,p,y);
		cout<<"\nSegment "<<i+1<<" X,Y,Z: "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<endl;
		cout<<"Segment "<<i+1<<" R,P,Y: "<<r<<", "<<p<<", "<<y<<endl;
	}
	
	return 0;
}
