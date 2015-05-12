/*
 * RosWrapper.cpp
 *
 *  Created on: May 10, 2014
 *      Author: mark
 */

#include "RosWrapper.h"

RosWrapper::RosWrapper() {
	// TODO Auto-generated constructor stub


}

RosWrapper::~RosWrapper() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXd RosWrapper::VectorArray2Matrix(int numD,
		std::vector<geometry_msgs::Vector3> vec)
{
	Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(vec.size(), numD);
	for (unsigned int i=0; i<vec.size(); ++i)
	{
		if (numD > 0)
			mat(i,0) = vec.at(i).x;
		if (numD > 1)
			mat(i,1) = vec.at(i).y;
		if (numD > 2)
			mat(i,2) = vec.at(i).z;
	}

	return mat;
}

bool RosWrapper::genPath(acl_msgs::GenPath::Request &req,
		acl_msgs::GenPath::Response &res)
{

	// parse inputs
	int N = req.N;
	int numD = req.numD;
	double R = req.R;
	double T = req.T;
	double h = req.h;

	Eigen::MatrixXd p0 = VectorArray2Matrix(numD, req.p0);
	Eigen::MatrixXd v0 = VectorArray2Matrix(numD, req.v0);
	Eigen::MatrixXd a0 = VectorArray2Matrix(numD, req.a0);
	Eigen::MatrixXd p1 = VectorArray2Matrix(numD, req.p1);
	Eigen::MatrixXd v1 = VectorArray2Matrix(numD, req.v1);
	Eigen::MatrixXd a1 = VectorArray2Matrix(numD, req.a1);
	Eigen::MatrixXd obstacles = VectorArray2Matrix(numD, req.obstacles);

    //code goes here
    //PathPlanner pp(N,numD,h, T, R);
	std::cout << "---------------------------------------------------------- "<< std::endl;
	std::cout << "------- Generating path for " << N << " agents------------ "<< std::endl;

	path_planner_array pp(N,numD,h, T, R);
    bool ifCon;
    double h_actual = pp.generatePath(p0, v0, a0, p1, v1, a1,
    		obstacles.transpose(), ifCon);

	for (int i=0; i<N; i++)
	{
		acl_msgs::Trajectory trajectory;

		std::vector<double> px, py, pz, vx, vy, vz, ax, ay, az, jx, jy, jz;
		pp.getPosition(i,0,px);
		pp.getPosition(i,1,py);
		pp.getVelocity(i,0,vx);
		pp.getVelocity(i,1,vy);
		pp.getAccel(i,0,ax);
		pp.getAccel(i,1,ay);
		pp.getJerk(i,0,jx);
		pp.getJerk(i,1,jy);

		if (numD > 2)
		{
			pp.getPosition(i, 2, pz);
			pp.getVelocity(i, 2, vz);
			pp.getAccel(i, 2, az);
			pp.getJerk(i,2,jz);
		}

		ros::Time cur_time(0);
		for (unsigned int j=0; j<px.size(); j++)
		{
			geometry_msgs::Vector3Stamped pos, vel, acc, jerk;
			if (j>0)
				cur_time += ros::Duration(h_actual);
			pos.header.stamp = cur_time;
			pos.vector.x = px.at(j);
			pos.vector.y = py.at(j);
			vel.header = pos.header;
			vel.vector.x = vx.at(j);
			vel.vector.y = vy.at(j);
			acc.header = pos.header;
			acc.vector.x = ax.at(j);
			acc.vector.y = ay.at(j);
			jerk.header = pos.header;
			jerk.vector.x = jx.at(j);
			jerk.vector.y = jy.at(j);
			//std::cout << "jerk x: " << jx.at(j) << std::endl;

			if (numD > 2)
			{
				pos.vector.z = pz.at(j);
				vel.vector.z = vz.at(j);
				acc.vector.z = az.at(j);
				jerk.vector.z = jz.at(j);
			}
			trajectory.pos.push_back(pos);
			trajectory.vel.push_back(vel);
			trajectory.acc.push_back(acc);
			trajectory.jerk.push_back(jerk);
		}
		res.trajectories.push_back(trajectory);
	}

	res.converged = ifCon;

	return true;
}
