/*
 * path_planner.cpp
 *
 *  Created on: May 7, 2014
 *      Author: mark cutler
 */

#include "path_planner.h"

/**
 * Path Planner constructor
 * @param N Number of vehicles
 * @param numD Number of dimensions (2 for ground vehicles, 3 for air)
 * @param h Time step size (typically somewhere between 0.02 and 0.2 seconds)
 * @param T Total time for maneuver
 */
PathPlanner::PathPlanner(int numD, double h, double T, double R)
{

	this->numD = numD;
	this->R = R;
	K = T / h + 1;
	this->h = h;
	timeScalingFactor = h;

	this->corner_x = -2.5;
	this->corner_y = -1.8;
	this->pole_x_min = -3.9;
	this->pole_x_max = -2.6;
	this->pole_y_min = -5.0;
	this->pole_y_max = -3.7;

	// Default values
	std::vector<double> pmax, pmin, amax, amin, jmax, jmin;
	pmax.push_back(2.5);
	pmax.push_back(2.5);
	pmin.push_back(-7.5);
	pmin.push_back(-6.9);
	setPositionBounds(pmax,pmin);
	amax.push_back(4);
	amax.push_back(4);
	amin.push_back(-4);
	amin.push_back(-4);
	setAccelBounds(amax,amin);
	jmax.push_back(4);
	jmax.push_back(4);
	jmin.push_back(-4);
	jmin.push_back(-4);
	setJerkBounds(jmax,jmin);

}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::setPositionBounds(std::vector<double> pmax, std::vector<double> pmin)
{
	p_max = Eigen::VectorXd::Map(pmax.data(),pmax.size());
	p_min = Eigen::VectorXd::Map(pmin.data(),pmin.size());

}

void PathPlanner::setVelocityBounds(std::vector<double> vmax, std::vector<double> vmin)
{
	v_max = Eigen::VectorXd::Map(vmax.data(),vmax.size());
	v_min = Eigen::VectorXd::Map(vmin.data(),vmin.size());
}

void PathPlanner::setAccelBounds(std::vector<double> amax, std::vector<double> amin)
{
	a_max = Eigen::VectorXd::Map(amax.data(),amax.size());
	a_min = Eigen::VectorXd::Map(amin.data(),amin.size());
}

void PathPlanner::setJerkBounds(std::vector<double> jmax, std::vector<double> jmin)
{
	j_max = Eigen::VectorXd::Map(jmax.data(),jmax.size());
	j_min = Eigen::VectorXd::Map(jmin.data(),jmin.size());
}

bool PathPlanner::getPosition(unsigned int dimension, std::vector<double> & pos)
{
	// sanity checks here
	if (dimension < numD){
		pos.clear();
		for (unsigned int i=0; i<K; ++i){
			pos.push_back((double) p_final(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting position for dimension that does not exist" << std::endl;
		return false;
	}

}

bool PathPlanner::getVelocity(unsigned int dimension, std::vector<double> & vel)
{
	// sanity checks here
	if (dimension < numD){
		vel.clear();
		for (unsigned int i=0; i<K; ++i){
			vel.push_back((double) v_final(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting velocity for dimension that does not exist" << std::endl;
		return false;
	}

}

bool PathPlanner::getAccel(unsigned int dimension, std::vector<double> & acc)
{
	// sanity checks here
	if (dimension < numD){
		acc.clear();
		for (unsigned int i=0; i<K; ++i){
			acc.push_back((double) a_final(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting acceleration for dimension that does not exist" << std::endl;
		return false;
	}

}

/**
 * Solve quadratic program of the form
 * 		min_x 1/2*x'*Q*x + c'*x
 * 		s.t. A*x <= b
 * 			 Aeq*x = beq
 * 			 lb <= x <= ub
 * @param Q
 * @param c
 * @param A
 * @param b
 * @param Aeq
 * @param beq
 * @param lb
 * @param ub
 * @param x Reference to optimal solution
 * @return Optimal objective function value if solved, +Inf otherwise
 */
double PathPlanner::solveQuadraticProgram(Eigen::MatrixXd Q, Eigen::VectorXd c,
		Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd Aeq,
		Eigen::VectorXd beq, Eigen::VectorXd lb, Eigen::VectorXd ub,
		Eigen::VectorXd & x)
{
	double obj_val = +MSK_INFINITY;

	// Combine A and Aeq constraints together
	Eigen::MatrixXd A_all = Eigen::MatrixXd::Zero(A.rows() + Aeq.rows(),
			Aeq.cols());
	A_all << A, Aeq;
	Eigen::VectorXd buc = Eigen::VectorXd::Zero(b.rows() + beq.rows());
	buc << b, beq;
	Eigen::VectorXd blc = Eigen::VectorXd::Zero(buc.rows());
	blc << -MSK_INFINITY * Eigen::VectorXd::Ones(b.rows()), beq;

	// number of variables, constraints, and nonzero elements in Q
	int numvar = Q.rows();
	int numcon = A_all.rows();
	Q = Q.triangularView<Eigen::Lower>(); // convert to dense lower triangular matrix
	Eigen::SparseMatrix<double> Q_sparse = Q.sparseView();
	int numQnz = Q_sparse.nonZeros();
	//std::cout << "numQnz: " << numQnz << std::endl;

	MSKint32t qsubi[numQnz];
	MSKint32t qsubj[numQnz];
	double qval[numQnz];

	MSKint32t i, j;
	double xx[numvar];

	MSKenv_t env = NULL;
	MSKtask_t task = NULL;
	MSKrescodee r;

	/* Create the mosek environment. */
	r = MSK_makeenv(&env, NULL);

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		if (r == MSK_RES_OK)
		{
			//r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

			/* Append 'numcon' empty constraints.
			 The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, numcon);

			/* Append 'numvar' variables.
			 The variables will initially be fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numvar);

			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);

			/* Add constraints and linear term */
			int cnt = 0;
			for (j = 0; j < numvar && r == MSK_RES_OK; ++j)
			{
				/* Set the linear term c_j in the objective.*/
				if (r == MSK_RES_OK)
					r = MSK_putcj(task, j, (double) c(j));

				/* Set the bounds on variable j.
				 blx[j] <= x_j <= bux[j] */
				if (r == MSK_RES_OK)
				{
					if (lb(j) <= -MSK_INFINITY and ub(j) >= +MSK_INFINITY)
						r = MSK_putvarbound(task, j, MSK_BK_FR, -MSK_INFINITY,
								+MSK_INFINITY);
					else if (lb(j) <= -MSK_INFINITY)
						r = MSK_putvarbound(task, j, MSK_BK_UP, -MSK_INFINITY,
								(double) ub(j));
					else if (ub(j) >= +MSK_INFINITY)
						r = MSK_putvarbound(task, j, MSK_BK_LO, (double) lb(j),
								+MSK_INFINITY);
					else if (lb(j) == ub(j))
						r = MSK_putvarbound(task, j, MSK_BK_FX, (double) lb(j),
								(double) ub(j));
					else
						r = MSK_putvarbound(task, j, MSK_BK_RA, (double) lb(j),
								(double) ub(j));
				}

				/* Input column j of A */
				int aptrb = cnt;
				Eigen::SparseVector<double> Aj = A_all.col(j).sparseView();
				int colnz = Aj.nonZeros();
				double ajval[colnz];
				MSKint32t ajsub[colnz];
				// iterate over the column of A
				int cnt_inner = 0;
				for (Eigen::SparseVector<double>::InnerIterator it(Aj); it;
						++it)
				{
					ajval[cnt_inner] = it.value();
					ajsub[cnt_inner] = it.index();
					cnt_inner++;
					cnt++;
				}
				if (r == MSK_RES_OK)
					r = MSK_putacol(task, j, cnt - aptrb, ajsub, ajval);

			}

			/* Set the bounds on constraints.
			 for i=1, ...,numcon : blc[i] <= constraint i <= buc[i] */
			for (i = 0; i < numcon && r == MSK_RES_OK; ++i)
			{
				MSKboundkeye key = MSK_BK_UP;
				if (blc[i] == buc[i])
					key = MSK_BK_FX;
				r = MSK_putconbound(task, i, key, (double) blc[i],
						(double) buc[i]);
			}

			if (r == MSK_RES_OK)
			{
				/*
				 * The lower triangular part of the Q
				 * matrix in the objective is specified.
				 */
				int cnt = 0;
				for (i = 0; i < Q_sparse.outerSize() && r == MSK_RES_OK; ++i)
				{
					for (Eigen::SparseMatrix<double>::InnerIterator it(Q_sparse,
							i); it; ++it)
					{
						if (it.row() >= i) {
							qsubi[cnt] = it.row();
							qsubj[cnt] = it.col();
							qval[cnt] = it.value();
							//std::cout << qsubi[cnt] << ", " << qsubj[cnt] << ", " << qval[cnt] << std::endl;
							cnt++;
						}
					}
				}

				/* Input the Q for the objective. */
				r = MSK_putqobj(task, numQnz, qsubi, qsubj, qval);
			}

			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				 about the solution for debugging purposes*/
				//MSK_solutionsummary(task, MSK_STREAM_MSG);

				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;

					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

					/* Request the interior solution. */
					MSK_getxx(task, MSK_SOL_ITR, xx);
					MSK_getprimalobj(task, MSK_SOL_ITR, &obj_val);

					x = Eigen::VectorXd::Map(xx, numvar);

				}
				else
				{
					printf("Error while optimizing.\n");
				}
			}

			if (r != MSK_RES_OK)
			{
				/* In case of an error print error code and description. */
				char symname[MSK_MAX_STR_LEN];
				char desc[MSK_MAX_STR_LEN];

				printf("An error occurred while optimizing.\n");
				MSK_getcodedesc(r, symname, desc);
				printf("Error %s - '%s'\n", symname, desc);
			}
		}
		MSK_deletetask(&task);
	}
	MSK_deleteenv(&env);

	return obj_val;

}

void PathPlanner::getA(Eigen::MatrixXd p_init, Eigen::MatrixXd v_init,
		Eigen::MatrixXd & A, Eigen::VectorXd & b)
{
	Eigen::MatrixXd A_pos, A_accel, A_jerk;
	Eigen::VectorXd b_pos, b_accel, b_jerk;
	getApos(p_init, v_init, A_pos, b_pos);

	A.resize(A_pos.rows(), A_pos.cols());
	b.resize(b_pos.rows());
	A.resize(A_pos.rows(), A_pos.cols());
	b.resize(b_pos.rows());
	A << A_pos;
	b << b_pos;
}

void PathPlanner::getAeq(Eigen::VectorXd p_init, Eigen::VectorXd v_init,
		Eigen::VectorXd a_init, Eigen::VectorXd p_final,
		Eigen::VectorXd v_final, Eigen::VectorXd a_final, Eigen::MatrixXd & Aeq,
		Eigen::VectorXd & beq)
{

		for (unsigned int i = 0; i < numD; i++)
		{
			Eigen::MatrixXd Aeq_tmp;
			Eigen::Vector4d beq_tmp;
			getAeqVehicle((double) p_init(i), (double) v_init(i),
					(double) a_init(i), (double) p_final(i),
					(double) v_final(i), (double) a_final(i), Aeq_tmp,
					beq_tmp);

			Eigen::MatrixXd Aeq_new = blkdiag(Aeq, Aeq_tmp);
			Aeq.resize(Aeq_new.rows(), Aeq_new.cols());
			Aeq << Aeq_new;

			Eigen::VectorXd beq_new(beq.rows() + beq_tmp.rows());
			beq_new << beq, beq_tmp;
			beq.resize(beq_new.rows());
			beq << beq_new;

		}
}

void PathPlanner::propagateStates(Eigen::VectorXd p0, Eigen::VectorXd v0,
		Eigen::VectorXd accel, Eigen::MatrixXd & p,
		Eigen::MatrixXd & v, Eigen::MatrixXd & a,
		Eigen::MatrixXd & j)
{


		Eigen::VectorXd tmp1 = accel.segment(0, numD * K); // x,y,(z) accelerations for vehicle n
		a = Eigen::Map<Eigen::MatrixXd>(tmp1.data(), K,
				numD).transpose();

		// Assign initial position and velocities
		p.setZero(numD, K);
		v.setZero(numD, K);
		j.setZero(numD, K);
		p.col(0) = p0.transpose();
		v.col(0) = v0.transpose();

		// Propagate
		for (unsigned int k = 1; k < K; ++k)
		{
			v.col(k) = v.col(k - 1) + h * a.col(k - 1);
			p.col(k) = p.col(k - 1) + h * v.col(k - 1)
					+ h * h / 2.0 * a.col(k - 1);
			j.col(k) = (a.col(k) - a.col(k - 1))/h;
		}
		//j.col(K-1) = Eigen::VectorXd::Zero(numD); // force last jerk element to be zero

}

void PathPlanner::collisionConstraints2Vehicles(Eigen::MatrixXd p1,
		Eigen::MatrixXd p2, Eigen::VectorXd v1_init, Eigen::VectorXd v2_init,
		Eigen::MatrixXd & A1, Eigen::MatrixXd & A2, Eigen::VectorXd & b)
{
	A1 = Eigen::MatrixXd::Zero(K, numD * K);
	A2 = Eigen::MatrixXd::Zero(K, numD * K);
	b = Eigen::VectorXd::Zero(K);

	for (unsigned int k = 1; k < K; ++k)
	{
		Eigen::VectorXd eta = calcEta(p1.col(k), p2.col(k));
		Eigen::VectorXd p1_0 = p1.col(0) + h * k * v1_init;
		Eigen::VectorXd p2_0 = p2.col(0) + h * k * v2_init;
		b(k) = R + eta.dot(p2_0 - p1_0);
		for (unsigned int dim = 0; dim < numD; ++dim)
		{
			for (unsigned int ii = 0; ii < k; ++ii)
			{
				A1(k, ii + dim * K) = eta(dim) * h * h / 2.0
						* (2 * (k + 1) - (2 * (ii + 1) + 1));
			}
		}
	}
	A2 << -A1;

	// negate A1, A2, and b since these constraints dictate Ax>=b
	A1 *= -1.0;
	A2 *= -1.0;
	b *= -1.0;
}

Eigen::VectorXd PathPlanner::calcEta(Eigen::VectorXd p1, Eigen::VectorXd p2)
{
	Eigen::VectorXd diff = p1 - p2;
	return diff / diff.norm();
}

void PathPlanner::getAeqVehicle(double p_init, double v_init, double a_init,
		double p_final, double v_final, double a_final, Eigen::MatrixXd & Aeq,
		Eigen::Vector4d & beq)
{

	Aeq = Eigen::MatrixXd::Zero(4, K);
	beq = Eigen::Vector4d::Zero();

	// Initial position and initial velcotiy are implicit

	// Initial Acceleration -- first row of Aeq
	Aeq(0, 0) = 1;
	beq(0) = a_init;

	// Final Position -- 2nd row
	for (unsigned int i = 0; i < K - 1; i++)
		Aeq(1, i) = h * h / 2.0 * (2 * K - (2 * (i + 1) + 1));
	beq(1) = p_final - p_init - h * (K - 1) * v_init;

	// Final Velocity -- 3rd row
	Aeq.row(2) = h * Eigen::VectorXd::Ones(K).transpose();
	Aeq(2, K - 1) = 0.0;
	beq(3) = v_final - v_init;

	// Final Acceleration -- 4th row
	Aeq(3, K - 1) = 1.0;
	beq(3) = a_final;

}

void PathPlanner::getAjerk(Eigen::MatrixXd & A, Eigen::VectorXd & b)
{
		for (unsigned int j = 0; j < numD; j++)
		{
			Eigen::MatrixXd A_tmp(2 * (K - 1), K);
			Eigen::VectorXd b_tmp(2 * (K - 1));
			getAjerkVehicle((double) j_max(j), (double) j_min(j), A_tmp, b_tmp);

			Eigen::MatrixXd A_new = blkdiag(A, A_tmp);
			A.resize(A_new.rows(), A_new.cols());
			A << A_new;

			Eigen::VectorXd b_new(b.rows() + b_tmp.rows());
			b_new << b, b_tmp;
			b.resize(b_new.rows());
			b << b_new;

		}
}

void PathPlanner::getAjerkVehicle(double j_max, double j_min,
		Eigen::MatrixXd & A, Eigen::VectorXd & b)
{
	Eigen::MatrixXd A_jerk = Eigen::MatrixXd::Identity(K - 1, K);
	Eigen::VectorXd b_jerk_max = j_max * Eigen::VectorXd::Ones(K - 1);
	Eigen::VectorXd b_jerk_min = j_min * Eigen::VectorXd::Ones(K - 1);

	for (unsigned int i = 0; i < K - 1; i++)
	{
		A_jerk(i, i + 1) = -1.0/h;
	}
	//A_jerk = A_jerk / h;
	// std::cout << "a_jer" << A_jerk << std::endl;
	A << A_jerk, -A_jerk;
	b << b_jerk_max, -b_jerk_min;
}

void PathPlanner::getApos(Eigen::VectorXd p_init, Eigen::VectorXd v_init, Eigen::MatrixXd & A,
		Eigen::VectorXd & b)
{

		for (unsigned int i = 0; i < numD; i++)
		{
			Eigen::MatrixXd A_tmp(2 * K, K);
			Eigen::VectorXd b_tmp(2 * K);
			getAposVehicle((double) p_max(i), (double) p_min(i),
					(double) p_init(i), (double) v_init(i), A_tmp, b_tmp);

			Eigen::MatrixXd A_new = blkdiag(A, A_tmp);
			A.resize(A_new.rows(), A_new.cols());
			A << A_new;

			Eigen::VectorXd b_new(b.rows() + b_tmp.rows());
			b_new << b, b_tmp;
			b.resize(b_new.rows());
			b << b_new;

		}
}

void PathPlanner::getAposVehicle(double p_max, double p_min, double p_init,
		double v_init, Eigen::MatrixXd & A, Eigen::VectorXd & b)
{
	// set the size of the inequality constraints
	Eigen::MatrixXd A_pos = Eigen::MatrixXd::Zero(K, K);
	Eigen::VectorXd b_pos = Eigen::VectorXd::Zero(K);
	Eigen::VectorXd ones = Eigen::VectorXd::Ones(K);

	for (unsigned int i = 1; i < K; i++)
	{
		for (unsigned int j = 0; j < i-1; j++)
		{
			A_pos(i, j) = h * h / 2.0 * (2 * (i + 1) - (2 * (j + 1) + 1));
			//std::cout << A_pos(i,j) << std::endl;
		}
		b_pos(i) = h * i * v_init;
	}
	Eigen::VectorXd b_pos_max = b_pos + p_max * ones - p_init * ones;
	Eigen::VectorXd b_pos_min = b_pos + p_min * ones - p_init * ones;

	//std::cout << A_pos << std::endl;

	A << A_pos, -A_pos;
	b << b_pos_max, -b_pos_min;
}

// Creates a block diagonal matrix out of the matrices A and B.  Should work
// the same as MATLAB's blkdiag function
Eigen::MatrixXd PathPlanner::blkdiag(Eigen::MatrixXd A, Eigen::MatrixXd B)
{
	if (A.rows() == 0 or A.cols() == 0)
		return B;
	else if (B.rows() == 0 or B.cols() == 0)
		return A;
	else
	{
		Eigen::MatrixXd C(A.rows() + B.rows(), A.cols() + B.cols());
		Eigen::MatrixXd z1 = Eigen::MatrixXd::Zero(A.rows(), B.cols());
		Eigen::MatrixXd z2 = Eigen::MatrixXd::Zero(B.rows(), A.cols());

		C << A, z1, z2, B;
		return C;
	}
}


void PathPlanner::setPosObs( std::vector <Eigen::MatrixXd> POS_OBS)
{
	this->pos_obs = POS_OBS;
	return;

}

void PathPlanner::clearPosObs()
{
	this->pos_obs.clear();
	return;
}


void PathPlanner::computeTimeScaleAgent()
{
		double ai_max = fabs(a_final.maxCoeff());
		double ji_max = fabs(j_final.maxCoeff());
		double scale_acc = 1;
		double scale_jerk = 1;

		if (ai_max > 0)
			scale_acc = a_max(0) / ai_max;
		if (ji_max > 0)
			scale_jerk = j_max(0) / ji_max;

		if (scale_acc < scale_jerk)
			this->timeScalingFactor = scale_acc;
		else
			this->timeScalingFactor = scale_jerk;
		std::cout << this->timeScalingFactor << std::endl;
	std::cout << std::endl << std::endl;
	return;
}

double PathPlanner::timeScaleAgent(double scalingFactor, Eigen::VectorXd p0, Eigen::VectorXd v0)
{

	this->x = this->x * scalingFactor;
	this->h = this->h / sqrt(scalingFactor);
	this->propagateStates(p0, v0, this->x, p_final, v_final, a_final, j_final);
	return this->h;
}

double PathPlanner::getTimeScalingFactor()
{
	return this->timeScalingFactor;
}

void PathPlanner::getAllConstr_tight(Eigen::MatrixXd & A, Eigen::VectorXd & b, Eigen::VectorXi & entry)
{

	// see Matlab script
	double c = 2.0;
	int counter = 0;
	A = this->A_old;
	b = this->b_old;

	for (unsigned int j = 0; j < this->K; j++)
	{
		int corner_constr = 0;
		int pole_constr = 0;
		int col_constr = 0;

		if (p_final(0,j)<this->corner_x && p_final(1,j)>this->corner_y)
			corner_constr = 1;

		if (p_final(0,j)>this->pole_x_min && p_final(0,j)< this->pole_x_max
				&& p_final(1,j)>this->pole_y_min && p_final(1,j)< this->pole_y_max)
			pole_constr = 1;

		for (unsigned int i = 0; i < this->pos_obs.size();i++)
		{
			Eigen::VectorXd diffVec = p_final.col(j) - this->pos_obs[i].col(j);
			if ( diffVec.norm() < this->R)
			{
				//std::cout << "constraint violated " << diffVec.norm() <<  "< "  << this->R<< std::endl;
			    col_constr = 1;
			    break;
			}
		}

		for (unsigned int i = 0; i < this->static_obs.size();i++)
		{
			Eigen::VectorXd diffVec = p_final.col(j) - this->static_obs[i];
//			std::cout << p_final.col(j) << std::endl;
//			std::cout << static_obs[i] << std::endl;
//			std::cout << diffVec << std::endl << std::endl;
			if ( diffVec.norm() < this->R)
			{
				//std::cout << "constraint violated " << diffVec.norm() <<  "< "  << this->R<< std::endl;
			    col_constr = 1;
			    break;
			}
		}

		if ((counter < 1 && (corner_constr == 1 || pole_constr == 1 || col_constr == 1)) || entry[j] == 1)
		{
			// hack
			int jj;
			if (j>1 && entry[j] == 0)
				jj = j - 1;
			else
				jj = j;


			// update entry counter
			if (entry[j] == 0)
			{
				counter += 1;
				entry[j] = 1;
			}


			// corner
			double b_base = exp(c*((double) p_final(0,jj)-this->corner_x)) + exp(c*(this->corner_y- (double) p_final(1,jj)));

			double d_dx = c * exp(c*((double)p_final(0,jj)-this->corner_x));
			double d_dy = -c * exp(c*(this->corner_y-(double)p_final(1,jj)));

			Eigen::MatrixXd a_x;
			Eigen::MatrixXd a_y;
			Eigen::MatrixXd a_temp;
			double b_x, b_y, b_temp, a_max, a_min, scaling;

			a_x = this->A_old.row(j);
			b_x = - (b_old(j) - p_max(0));
			a_y = this->A_old.row(2*this->K + j);
			b_y = - (b_old(2*this->K + j) - p_max(1));

			// numerical scaling
			a_temp = - ((d_dx * a_x + d_dy * a_y));
			b_temp = (b_base + d_dx * (b_x - p_final(0,jj)) +
					d_dy * (b_y - p_final(1,jj))) - 2;

			a_max = a_temp.maxCoeff();
			a_min = a_temp.minCoeff();

			if (a_max < -a_min)
				a_max = -a_min;
			if (a_max > fabs(b_temp))
				scaling = a_max;
			else
				scaling = fabs(b_temp);

			if (scaling > 0.003)
			{
				A.conservativeResize(A.rows()+1, A.cols());
				A.row(A.rows()-1) = a_temp/scaling;
				b.conservativeResize(b.rows()+1);
				b[b.rows()-1] = b_temp / scaling;
				//std::cout << j << " constr added corner" << std::endl;
			}
			else
			{
				A.conservativeResize(A.rows()+1, A.cols());
				A.row(A.rows()-1) = a_temp/scaling;
				b.conservativeResize(b.rows()+1);
				b[b.rows()-1] = b_temp / scaling;
				//std::cout << j << " constr added corner" << std::endl;
			}

			// pole
			b_base = exp(c*((double) p_final(0,jj)-this->pole_x_max)) +
					+ exp(c*((double) p_final(1,jj)-this->pole_y_max))
					+ exp(c*(this->pole_x_min - (double) p_final(0,jj)))
					+ exp(c*(this->pole_y_min - (double) p_final(1,jj)));

			d_dx = c * exp(c*((double)p_final(0,jj)-this->pole_x_max))
					- c * exp(c*(this->pole_x_min - (double)p_final(0,jj)));
			d_dy = c * exp(c*((double)p_final(1,jj)-this->pole_y_max))
					- c * exp(c*(this->pole_y_min - (double)p_final(1,jj)));

			a_x = this->A_old.row(j);
			b_x = - (b_old(j) - p_max(0));
			a_y = this->A_old.row(2*this->K + j);
			b_y = - (b_old(2*this->K + j) - p_max(1));

			// numerical scaling
			a_temp = - ((d_dx * a_x + d_dy * a_y));
			b_temp = (b_base + d_dx * (b_x - p_final(0,jj)) +
					d_dy * (b_y - p_final(1,jj))) - 2.5;

			a_max = a_temp.maxCoeff();
			a_min = a_temp.minCoeff();

			if (a_max < -a_min)
				a_max = -a_min;
			if (a_max > fabs(b_temp))
				scaling = a_max;
			else
				scaling = fabs(b_temp);

			if (scaling > 0.003)
			{
				A.conservativeResize(A.rows()+1, A.cols());
				A.row(A.rows()-1) = a_temp/scaling;
				b.conservativeResize(b.rows()+1);
				b[b.rows()-1] = b_temp / scaling;
				//std::cout << j << " constr added pole" << std::endl;
			}
			else
			{
				A.conservativeResize(A.rows()+1, A.cols());
				A.row(A.rows()-1) = a_temp/scaling;
				b.conservativeResize(b.rows()+1);
				b[b.rows()-1] = b_temp / scaling;
				//std::cout << j << " constr added pole" << std::endl;
			}


			// collision
			for (unsigned int i = 0; i < this->pos_obs.size(); i++)
			{
				b_base = pow( (double) p_final(0,jj) - (double) pos_obs[i](0,jj),2)
						+  pow( (double) p_final(1,jj) - (double) pos_obs[i](1,jj),2) - pow(R+0.2,2);
				d_dx = 2 * ((double) p_final(0,jj) - (double) pos_obs[i](0,jj));
				d_dy = 2 * ((double) p_final(1,jj) - (double) pos_obs[i](1,jj));

				a_x = this->A_old.row(j);
				b_x = - (b_old(j) - p_max(0));
				a_y = this->A_old.row(2*this->K + j);
				b_y = - (b_old(2*this->K + j) - p_max(1));

				// numerical scaling
				a_temp = - ((d_dx * a_x + d_dy * a_y));
				b_temp = (b_base + d_dx * (b_x - p_final(0,jj)) +
						d_dy * (b_y - p_final(1,jj)));

				a_max = a_temp.maxCoeff();
				a_min = a_temp.minCoeff();

				if (a_max < -a_min)
					a_max = -a_min;
				if (a_max > fabs(b_temp))
					scaling = a_max;
				else
					scaling = fabs(b_temp);

//				if (scaling > 0.003)
//				{
					A.conservativeResize(A.rows()+1, A.cols());
					A.row(A.rows()-1) = a_temp/scaling;
					b.conservativeResize(b.rows()+1);
					b[b.rows()-1] = b_temp / scaling;
					//std::cout << j << " constr added inter-agent" << std::endl;
//				}
//				else
//				{
//					A.conservativeResize(A.rows()+1, A.cols());
//					A.row(A.rows()-1) = a_temp/scaling;
//					b.conservativeResize(b.rows()+1);
//					b[b.rows()-1] = b_temp / scaling;
//					//std::cout << j << " constr added inter-agent" << std::endl;
//				}

			}

			// collision w/ static obstacles
			for (unsigned int i = 0; i < this->static_obs.size(); i++)
			{
				b_base = pow( (double) p_final(0,jj) - (double) static_obs[i](0), 2)
						+  pow( (double) p_final(1,jj) - (double) static_obs[i](1), 2) - pow(R+0.2,2);
				d_dx = 2 * ((double) p_final(0,jj) - (double) static_obs[i](0));
				d_dy = 2 * ((double) p_final(1,jj) - (double) static_obs[i](1));

				a_x = this->A_old.row(j);
				b_x = - (b_old(j) - p_max(0));
				a_y = this->A_old.row(2*this->K + j);
				b_y = - (b_old(2*this->K + j) - p_max(1));

				// numerical scaling
				a_temp = - ((d_dx * a_x + d_dy * a_y));
				b_temp = (b_base + d_dx * (b_x - p_final(0,jj)) +
						d_dy * (b_y - p_final(1,jj)));

				a_max = a_temp.maxCoeff();
				a_min = a_temp.minCoeff();

				if (a_max < -a_min)
					a_max = -a_min;
				if (a_max > fabs(b_temp))
					scaling = a_max;
				else
					scaling = fabs(b_temp);

				A.conservativeResize(A.rows()+1, A.cols());
				A.row(A.rows()-1) = a_temp/scaling;
				b.conservativeResize(b.rows()+1);
				b[b.rows()-1] = b_temp / scaling;


			}


		}



	}


    return;
}


void PathPlanner::solveSingleAgent(Eigen::VectorXd p0, Eigen::VectorXd v0,
	Eigen::VectorXd a0, Eigen::VectorXd p1, Eigen::VectorXd v1,
	Eigen::VectorXd a1, int maxIter, bool & ifCon)
{

	// Get constraints
	Eigen::MatrixXd A, Aeq;
	Eigen::VectorXd b, beq;
	getAeq(p0, v0, a0, p1, v1, a1, Aeq, beq);
	getA(p0, v0, A, b);
	this->A_old = A;
	this->b_old = b;

	// Get Q and c
	Eigen::MatrixXd Q_accel = Eigen::MatrixXd::Identity(numD * K, numD * K);


	Eigen::MatrixXd Q_jerk_1d = Eigen::MatrixXd::Identity(K, K);
	Q_jerk_1d.diagonal(-1) = -1*Eigen::VectorXd::Ones(K - 1);
	Q_jerk_1d(0, 0) = 0.0;
	Q_jerk_1d = 1/h*Q_jerk_1d.transpose()*Q_jerk_1d;
	Eigen::MatrixXd Q_jerk;
	for (int i=0; i<numD; i++)
	{
		Q_jerk = blkdiag(Q_jerk, Q_jerk_1d);
	}


	Eigen::VectorXd c = Eigen::VectorXd::Zero(numD * K);
	Eigen::VectorXd lb = -1.0e30 * Eigen::VectorXd::Ones(numD * K);
	Eigen::VectorXd ub = +1.0e30 * Eigen::VectorXd::Ones(numD * K);

	Eigen::MatrixXd Q = 2.0*Q_jerk;
//	Eigen::MatrixXd Q = 2.0*Q_accel;
//	std::cout << Q << std::endl;
//	std::cout << "jr: " << Q_jerk.rows() << " jc: " << Q_jerk.cols() << std::endl;
//	std::cout << "ar: " << Q_accel.rows() << " ac: " << Q_accel.cols() << std::endl;
//	std::cout << "A:" << std::endl;
//	std::cout << A << std::endl;
//	std::cout << "b:" << std::endl;
//	std::cout << b << std::endl;
//	std::cout << "Aeq:" << std::endl;
//	std::cout << Aeq << std::endl;
//	std::cout << "beq:" << std::endl;
//	std::cout << beq << std::endl;

	double obj_val = solveQuadraticProgram(Q, c, A, b, Aeq, beq, lb, ub, this->x);

	propagateStates(p0, v0, this->x, p_final, v_final, a_final, j_final);

	std::cout << p_final << std::endl;

	Eigen::VectorXi entry = Eigen::VectorXi::Zero(K);

	//std::cout << " -------- entering mainloop ------------------- " << std::endl;
	for (int i = 0 ; i<maxIter-1; i++)
	{
		//std::cout << "hello0" << std::endl;
		this->getAllConstr_tight(A, b, entry);
		obj_val = this->solveQuadraticProgram(Q, c, A, b, Aeq, beq, lb, ub, x);

		//std::cout << " ~~~~~~~~~~~~~~~finished solving quadratic program~~~~~~~~~~~~~~~~~~~~~~~~ " << std::endl;
		//std::cout << " ---           iteration " << i << std::endl;
		Eigen::MatrixXd p_final_old = p_final;
		propagateStates(p0, v0, this->x, p_final, v_final, a_final, j_final);


		ifCon = this->ifConvergedPos(p_final,p_final_old, p1.transpose(), 0.05);
		// algorithm has converged
		if (ifCon == true)
		{
			std::cout<< "converged in step " << i << std::endl;
			this->computeTimeScaleAgent();
			return;
		}
	}
}


bool PathPlanner::ifConvergedPos(Eigen::MatrixXd pos, Eigen::MatrixXd pos_old, Eigen::MatrixXd p1, double tol)
{
	// final position

	std::cout << "pos: " << pos.col(pos.cols()-1) << std::endl;
	std::cout << "p1: " << p1 << std::endl;
	Eigen::VectorXd tmp = pos.col(pos.cols()-1) - p1.transpose();
	if (tmp.norm() > 0.1)
	{
		//std::cout << pos.col(pos.cols()-1) << std::endl;
		//std::cout << p1 << std::endl;
		//std::cout << "       final condition not satisfied " << std::endl;
		return false;
	}

	// for each step
	for (unsigned int i = 0 ; i<this->K; i++)
	{

		// current iteration results agree with past iteration
		Eigen::VectorXd diffTmp = pos.col(i) - pos_old.col(i);
		if (diffTmp.norm() > tol)
		{
			//std::cout << "       pos[" << i << "] changed by more than tol" << std::endl;
			return false;
		}

		// pole constraint
		if (pos(0,i) > this->pole_x_min && pos(0,i) < this->pole_x_max
				&& pos(1,i) > this->pole_y_min && pos(1,i) < this->pole_y_max)
		{
			//std::cout << "       pos[" << i << "] infeasible (in pole)" << std::endl;
			return false;
		}

		// infeasible corner constraint
		if (pos(0,i) < this->corner_x && pos(1,i) > this->corner_y)
		{
			//std::cout << "       pos[" << i << "] infeasible (in corner)" << std::endl;
			return false;
		}

		// collision with other agents
		for (unsigned int k = 0 ; k < this->pos_obs.size(); k++)
		{
			diffTmp = pos.col(i) - pos_obs[k].col(i);
			if (diffTmp.norm() < R)
			{
				//std::cout << pos.col(i).transpose() << std::endl;
				//std::cout << pos_obs[k].col(i).transpose() << std::endl;
				//std::cout << "       pos[" << i << "] infeasible (close to other agent)" << std::endl;
				return false;
			}
		}

		// collision with static obstacles
		for (unsigned int k=0; k<static_obs.size(); k++)
		{
			diffTmp = pos.col(i) - static_obs[k];
			if (diffTmp.norm() < R) // TODO: could make R a member of static obstacles so it can be different for different obstacles
				return false;
		}
	}

	return true;
}
