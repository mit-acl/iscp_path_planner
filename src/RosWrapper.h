/*
 * RosWrapper.h
 *
 *  Created on: May 10, 2014
 *      Author: mark
 */

#ifndef ROSWRAPPER_H_
#define ROSWRAPPER_H_

// ROS includes
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "path_planner.h"
#include "pathplannerarray.h"

#include "acl_msgs/GenPath.h"
#include "acl_msgs/Trajectory.h"

class RosWrapper {
public:
	RosWrapper();
	virtual ~RosWrapper();

	ros::ServiceServer service;

	bool genPath(acl_msgs::GenPath::Request &req,
			acl_msgs::GenPath::Response &res);

private:
	Eigen::MatrixXd VectorArray2Matrix(int numD,
			std::vector<geometry_msgs::Vector3> vec);
};

#endif /* ROSWRAPPER_H_ */
