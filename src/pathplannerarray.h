/*
 * pathplannerarray.h
 *
 *  Created on: Jul 14, 2014
 *      Author: steven
 */

#ifndef PATHPLANNERARRAY_H_
#define PATHPLANNERARRAY_H_

#include "path_planner.h"
#include <ctime>


class path_planner_array {
public:
	path_planner_array(int N, int numD, double h, double T, double R);
	virtual ~path_planner_array();
	bool getPosition(int vehicle, int dimension, std::vector<double> & pos);
	bool getVelocity(int vehicle, int dimension, std::vector<double> & vel);
	bool getAccel(int vehicle, int dimension, std::vector<double> & acc);
	bool getJerk(int vehicle, int dimension, std::vector<double> & jerk);
	double generatePath(Eigen::MatrixXd p0, Eigen::MatrixXd v0,
			Eigen::MatrixXd a0, Eigen::MatrixXd p1, Eigen::MatrixXd v1,
			Eigen::MatrixXd a1, Eigen::MatrixXd obstacles, bool &globalCon);

	bool isValidInput();
	void swapOrder();
    void heuristicOrder(Eigen::MatrixXd p0, Eigen::MatrixXd p1);
	double timeScale(Eigen::MatrixXd p0, Eigen::MatrixXd v0);

	unsigned int K; ///< number of discretizations
	unsigned int N; ///< number of vehicles
	unsigned int numD; ///< number of dimensions

	std::vector <Eigen::MatrixXd> p_final, v_final, a_final, j_final;
	std::vector <PathPlanner> agents;
	std::vector <int> order;
	std::vector <double> timeScalingFactors;
};

#endif /* PATHPLANNERARRAY_H_ */
