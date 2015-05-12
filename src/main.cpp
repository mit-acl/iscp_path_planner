#include <iostream>

#include "RosWrapper.h"


int main(int argc, char* argv[])
{
//
//	int N = 1;
//	int numD = 2;
//	double h = 0.25;
//	double T = 5.0;
//	double R = 0.8;
//
////	Eigen::Matrix2d p0, v0, a0, p1, v1, a1, obs;
////	p0 << -1, -6, -1, -3;
////	p1 << -1, -3, -1, -6;
//	Eigen::Vector2d p0, v0, a0, p1, v1, a1, obs;
//	p0 << 0, 0;
//	p1 << 0, -4;
//	v0.setZero();
//	a0.setZero();
//	v1.setZero();
//	a1.setZero();
//	obs << 0, -2;
//
//
//	path_planner_array pp(N,numD,h, T, R);
//    bool ifCon;
//    double h_actual = pp.generatePath(p0.transpose(), v0.transpose(), a0.transpose(),
//    		p1.transpose(), v1.transpose(), a1.transpose(), obs, ifCon);
//
//    if (ifCon)
//    {
//		for (int i=0; i<N; i++)
//		{
//			std::vector<double> px, py, pz, vx, vy, vz, ax, ay, az, jx, jy, jz;
//			pp.getPosition(i,0,px);
//			pp.getPosition(i,1,py);
//			pp.getVelocity(i,0,vx);
//			pp.getVelocity(i,1,vy);
//			pp.getAccel(i,0,ax);
//			pp.getAccel(i,1,ay);
//			pp.getJerk(i,0,jx);
//			pp.getJerk(i,1,jy);
//			std::cout << std::endl;
//			for (int j=0; j<ay.size(); j++)
//				std::cout << ax[j] << std::endl;
//			for (int j=0; j<ay.size(); j++)
//				std::cout << ay[j] << std::endl;
//		}
//    }
//
//    return ifCon;

	ros::init(argc, argv, "path_planner");
	ros::NodeHandle n;

	RosWrapper rw;

	rw.service = n.advertiseService("gen_path", &RosWrapper::genPath, &rw);

	ros::spin();

    return 0;
}



