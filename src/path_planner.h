/*
 * path_planner.h
 *
 *  Created on: May 7, 2014
 *      Author: mark cutler
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

#include "mosek.h" /* Include the MOSEK definition file. */

class PathPlanner
{
public:
	PathPlanner(int numD, double h, double T, double R);
	virtual ~PathPlanner();
	bool getPosition(unsigned int dimension, std::vector<double> & pos);
	bool getVelocity(unsigned int dimension, std::vector<double> & vel);
	bool getAccel(unsigned int dimension, std::vector<double> & acc);
	void setPositionBounds(std::vector<double> pmax, std::vector<double> pmin);
	void setVelocityBounds(std::vector<double> vmax, std::vector<double> vmin);
	void setAccelBounds(std::vector<double> amax, std::vector<double> amin);
	void setJerkBounds(std::vector<double> jmax, std::vector<double> jmin);

	// added by steven
	double corner_x;
	double corner_y;
	double pole_x_min;
	double pole_x_max;
	double pole_y_min;
	double pole_y_max;
	double getTimeScalingFactor();
	double timeScaleAgent(double scalingFactor,Eigen::VectorXd p0, Eigen::VectorXd v0);
	void solveSingleAgent( Eigen::VectorXd p0, Eigen::VectorXd v0,
	    Eigen::VectorXd a0, Eigen::VectorXd p1, Eigen::VectorXd v1,
	    Eigen::VectorXd a1, int maxIter, bool & ifCon);
	Eigen::MatrixXd p_final, v_final, a_final, j_final;
	std::vector<Eigen::MatrixXd> pos_obs, static_obs;

private:
	double h; ///< time step
	double R; ///< Safety distance that all vehicles must be a away from each other
	unsigned int K; ///< number of discretizations
	unsigned int numD; ///< number of dimensions
	Eigen::VectorXd p_max, p_min, v_max, v_min, a_max, a_min, j_max, j_min;


	double solveQuadraticProgram(Eigen::MatrixXd Q, Eigen::VectorXd c,
			Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd Aeq,
			Eigen::VectorXd beq, Eigen::VectorXd lb, Eigen::VectorXd ub,
			Eigen::VectorXd & x);
	void propagateStates(Eigen::VectorXd p0, Eigen::VectorXd v0,
		Eigen::VectorXd accel, Eigen::MatrixXd & p,
		Eigen::MatrixXd & v, Eigen::MatrixXd & a,
		Eigen::MatrixXd & j);

	void getApos(Eigen::VectorXd p_init, Eigen::VectorXd v_init, Eigen::MatrixXd & A,
			Eigen::VectorXd & b);
	void getAposVehicle(double p_max, double p_min, double p_init,
			double v_init, Eigen::MatrixXd & A, Eigen::VectorXd & b);
	void getAjerk(Eigen::MatrixXd & A, Eigen::VectorXd & b);
	void getAjerkVehicle(double j_max, double j_min, Eigen::MatrixXd & A,
			Eigen::VectorXd & b);
	void getAeq(Eigen::VectorXd p_init, Eigen::VectorXd v_init,
		Eigen::VectorXd a_init, Eigen::VectorXd p_final,
		Eigen::VectorXd v_final, Eigen::VectorXd a_final,
			Eigen::MatrixXd & Aeq, Eigen::VectorXd & beq);
	void getAeqVehicle(double p_init, double v_init, double a_init,
			double p_final, double v_final, double a_final,
			Eigen::MatrixXd & Aeq, Eigen::Vector4d & beq);
	void getA(Eigen::MatrixXd p_init, Eigen::MatrixXd v_init,
			Eigen::MatrixXd & A, Eigen::VectorXd & b);
	void collisionConstraints(std::vector<Eigen::MatrixXd> p,
			std::vector<Eigen::MatrixXd> v, Eigen::MatrixXd & A,
			Eigen::VectorXd & b);
	void collisionConstraints2Vehicles(Eigen::MatrixXd p1, Eigen::MatrixXd p2,
			Eigen::VectorXd v1_init, Eigen::VectorXd v2_init,
			Eigen::MatrixXd & A1, Eigen::MatrixXd & A2, Eigen::VectorXd & b);
	Eigen::VectorXd calcEta(Eigen::VectorXd p1, Eigen::VectorXd p2);
	Eigen::MatrixXd blkdiag(Eigen::MatrixXd A, Eigen::MatrixXd B);

	// added by steven
	void setPosObs( std::vector <Eigen::MatrixXd> POS_OBS);
	void clearPosObs();
	double timeScalingFactor;
	void computeTimeScaleAgent();

	void getAllConstr_tight(Eigen::MatrixXd & A, Eigen::VectorXd & b, Eigen::VectorXi & entry);
	Eigen::MatrixXd A_old, b_old;
	Eigen::VectorXd x;


	bool ifConvergedPos(Eigen::MatrixXd pos, Eigen::MatrixXd pos_old, Eigen::MatrixXd p1, double tol);

};

#endif /* PATH_PLANNER_H_ */
