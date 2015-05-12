/*
 * pathplannerarray.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: steven
 */

#include "pathplannerarray.h"
#include <math.h>
#include <sys/time.h>

double get_wall_time(){
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}



path_planner_array::path_planner_array(int N, int numD, double h, double T, double R) {
	// TODO Auto-generated constructor stub
	this->N = N;
	this->numD = numD;
	this->K = T / h + 1;

    // set deterministic random seed
    srand(1);

	// construct instances of pp for each agent
	for (int i = 0; i < N; i++)
	{
	    this->agents.push_back(PathPlanner(numD,h, T, R));
	    this->order.push_back(i);
	}

}



path_planner_array::~path_planner_array() {
	// TODO Auto-generated destructor stub
}


bool path_planner_array::getPosition(int vehicle, int dimension, std::vector<double> & pos)
{
	// sanity checks here
	if (vehicle < N and dimension < numD){
		pos.clear();
		Eigen::MatrixXd veh = p_final.at(vehicle);
		for (int i=0; i<K; ++i){
			pos.push_back((double) veh(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting position for vehicle or dimension that does not exist" << std::endl;
		return false;
	}

}

bool path_planner_array::getVelocity(int vehicle, int dimension, std::vector<double> & vel)
{
	// sanity checks here
	if (vehicle < N and dimension < numD){
		vel.clear();
		Eigen::MatrixXd veh = v_final.at(vehicle);
		for (unsigned int i=0; i<K; ++i){
			vel.push_back((double) veh(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting velocity for vehicle or dimension that does not exist" << std::endl;
		return false;
	}

}

bool path_planner_array::getAccel(int vehicle, int dimension, std::vector<double> & acc)
{
	// sanity checks here
	if (vehicle < N and dimension < numD){
		acc.clear();
		Eigen::MatrixXd veh = a_final.at(vehicle);
		for (unsigned int i=0; i<K; ++i){
			acc.push_back((double) veh(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting acceleration for vehicle or dimension that does not exist" << std::endl;
		return false;
	}

}

bool path_planner_array::getJerk(int vehicle, int dimension, std::vector<double> & jerk)
{
	// sanity checks here
	if (vehicle < N and dimension < numD){
		jerk.clear();
		Eigen::MatrixXd veh = j_final.at(vehicle);
		for (unsigned int i=0; i<K; ++i){
			jerk.push_back((double) veh(dimension,i));
		}
		return true;
	}
	else {
		std::cout << "Warning: requesting jerk for vehicle or dimension that does not exist" << std::endl;
		return false;
	}

}

double path_planner_array::generatePath(Eigen::MatrixXd p0, Eigen::MatrixXd v0,
			Eigen::MatrixXd a0, Eigen::MatrixXd p1, Eigen::MatrixXd v1,
			Eigen::MatrixXd a1, Eigen::MatrixXd obstacles, bool &globalCon)
{

	double begin_time = get_wall_time();
	globalCon = false;


	this->heuristicOrder(p0,p1);
	// mainloop
	for (int j = 0; j<10; j++)
	{
		std::cout << "in ordering "<< j << std::endl;
		//reset
		this->p_final.clear();
		this->v_final.clear();
		this->a_final.clear();
		this->j_final.clear();

		for (unsigned int k = 0; k < this->agents.size(); k++)
		{
			this->agents[k].pos_obs.clear();

			// add static obstacles to all agents (includes current static vehicles)
			this->agents[k].static_obs.clear();
			// TODO: figure out how to deal with no obstacles			
			//this->agents[k].static_obs.push_back(obstacles);
		}


		Eigen::MatrixXd rand_p0 = 0.001 * Eigen::MatrixXd::Random(this->agents.size(), this->numD);
		Eigen::MatrixXd rand_p1 = 0.001 * Eigen::MatrixXd::Random(this->agents.size(), this->numD);
		for (unsigned int i = 0; i < N; i++)
		{
			bool ifCon;
			std::cout << "     solving for " << i << "th agent with id " << this->order[i] << std::endl;
			this->agents[order[i]].solveSingleAgent(p0.row(order[i])+rand_p0.row(order[i]), v0.row(order[i]),
					a0.row(order[i]), p1.row(order[i])+ rand_p1.row(order[i]), v1.row(order[i]), a1.row(order[i]),
					K*2, ifCon);

			//this->agents[order[i]].solveSingleAgent(p0.row(order[i]), v0.row(order[i]),
			//		a0.row(order[i]), p1.row(order[i]), v1.row(order[i]), a1.row(order[i]),
			//	    3, ifCon);

			// add to obstacle list of its teammates
			for (unsigned int k = i+1; k < N; k++)
				this->agents[order[k]].pos_obs.push_back(this->agents[order[i]].p_final);

			// add to path planner array
			this->p_final.push_back(this->agents[order[i]].p_final);
			this->v_final.push_back(this->agents[order[i]].v_final);
			this->a_final.push_back(this->agents[order[i]].a_final);
			this->j_final.push_back(this->agents[order[i]].j_final);



			if (ifCon == false)
			{
				this->order[0] = this->order[i];
				break;
			}
			if (i == N-1 && ifCon ==1)
				globalCon = 1;

		}
		if (globalCon == 1)
			break;

		this->swapOrder();
	}

	// rescale
	double h = this->timeScale(p0, v0);
	double end_time = get_wall_time();
	double elapsed_secs = end_time-begin_time;
	std::cout << "***  time elapsed: " << elapsed_secs << std::endl;
	return h;

}

void path_planner_array::swapOrder()
{
	//std::cout << "in swapOrder " << std::endl;
	//for (int k = 0; k < this->order.size(); k++)
	//    std::cout << this->order[k] << '\t';
	//std::cout << std::endl;

	for (unsigned int i = 1; i < N; i++)
	{
		// generate a new index (cannot repeat)
		bool valid = false;
		int new_index;

		while (not valid)
		{
			valid = true;
			new_index = rand() % N;
		    for (int j = 0; j < i; j++)
		    	if (this->order[j] == new_index)
		    	{
		    		valid = false;
		    		break;
		    	}
	     }

		this->order[i] = new_index;

	}
	//for (int k = 0; k < this->order.size(); k++)
	//    std::cout << this->order[k] << '\t';
	//std::cout << std::endl;

	return;

}

void path_planner_array::heuristicOrder(Eigen::MatrixXd p0, Eigen::MatrixXd p1)
{
	//calculate distance between initial and final positions
    std::vector <double> dist;
    std::vector <double> tmp;
    for (unsigned int i = 0; i < N; i++)
    {
        Eigen::VectorXd diffVec = p1.row(i) - p0.row(i);
        dist.push_back(diffVec.norm());
        tmp.push_back(dist[i]);
    }

    //sort distance
    std::sort(dist.begin(), dist.end());

    //pull out indices corresponding to sorted distance
    for (unsigned int i =0; i < N; i++)
    {
        for (unsigned int j = 0; j < N; j++)
        {
            if(dist[N-1-i] == tmp[j])
            {
                tmp[j] = 9999;
                this->order[i] = j;
                break;
            }
        }
    }

	return;

}

double path_planner_array::timeScale(Eigen::MatrixXd p0, Eigen::MatrixXd v0)
{
	double max_scale = 9999;
	double h;

	// find the max allowed scaling factor
		for (unsigned int i = 0; i < this->agents.size(); i++)
		{
			if(this->agents[i].getTimeScalingFactor() <  max_scale)
				max_scale = this->agents[i].getTimeScalingFactor();
		}
		std::cout << max_scale << std::endl;

	// scale each agent accordings
	for (unsigned int i = 0; i < this->agents.size(); i++)
		h = this->agents[i].timeScaleAgent(max_scale, p0.row(i),v0.row(i));

	// reload results from each agent
	p_final.clear();
	v_final.clear();
	a_final.clear();
	j_final.clear();

	for (unsigned int i = 0; i < this->agents.size(); i++)
	{
	    this->p_final.push_back(this->agents[i].p_final);
	    this->v_final.push_back(this->agents[i].v_final);
	    this->a_final.push_back(this->agents[i].a_final);
	    this->j_final.push_back(this->agents[i].j_final);
	}

	return h;
}

// to be implementated later
bool path_planner_array::isValidInput()
{
	return true;
}
