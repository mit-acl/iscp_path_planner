# Setup MOSEK #
```
cd ~
wget http://download.mosek.com/stable/7.1.0.27/mosektoolslinux64x86.tar.bz2
tar xvf mosektoolslinux64x86.tar.bz2
```

get liscense file mosek.lic and put it in the mosek folder.

setup environment variable by adding the following lines to the `~/.bashrc`

```
export PATH=$HOME/mosek/7/tools/platform/linux64x86/bin:$PATH
export LD_LIBRARY_PATH=$HOME/mosek/7/tools/platform/linux64x86/bin:$LD_LIBRARY_PATH
```

# launching multi-quad demo

[Optional]
To run projector: 

0.1 run trajectory talker on Sikorsky
		roscd udp_project
		cd src
		python traj_visualizer_talker2.py
	
0.2 run projector code on Thor
		go to brett_workspace/udp_project/src
		python quad_projector.py


1. run visualization
	roslaunch raven_rviz rviz.launch config:=interactive 

2. start controllers (make sure 'sim' bit agrees with multi_vehicle_server.py)
	/***********************************************************************/        
	// c = MultiVehicleWaypointServer(['BQ01s', 'BQ02s', 'BQ03s', 'BQ04s'])
	/***********************************************************************/
	roslaunch quad_control control.launch veh:=BQ num:=01 sim:=1 iscp:=1
	roslaunch quad_control control.launch veh:=BQ num:=02 sim:=1 iscp:=1
	roslaunch quad_control control.launch veh:=BQ num:=03 sim:=1 iscp:=1
	roslaunch quad_control control.launch veh:=BQ num:=04 sim:=1 iscp:=1


3. run path_server
	rosrun iscp_path_planner multi_vehicle_path_server.py

4. run demo trajectories
	rosrun iscp_path_planner multi_vehicle_demo.py



********************************************************************************
debugging using path.pdf

0. install latex package
	apt-get install texlive-latex-extra
    apt-get install texlive
    install dvipng

1. run path planner service 
	rosrun iscp_path_planner path_planner

2. generate random test case
	rosrun iscp_path_planner path_planner_single.py

3. path.pdf file should be generated in directory from which 3 is ran
	square - starting position
	circle - intermediate waypoints

********************************************************************************
debugging using raven_rviz

1. run visualization
	roslaunch raven_rviz rviz.launch config:=interactive 

2. start controllers (make sure 'sim' bit agrees with multi_vehicle_server.py)
	/***********************************************************************/        
	// c = MultiVehicleWaypointServer(['BQ01s', 'BQ02s', 'BQ03s', 'BQ04s'])
	/***********************************************************************/
	roslaunch quad_control control.launch veh:=BQ num:=01 sim:=1 iscp:=1
	roslaunch quad_control control.launch veh:=BQ num:=02 sim:=1 iscp:=1
	roslaunch quad_control control.launch veh:=BQ num:=03 sim:=1 iscp:=1
	roslaunch quad_control control.launch veh:=BQ num:=04 sim:=1 iscp:=1


3. run path_server
	rosrun iscp_path_planner multi_vehicle_path_server.py

4. run path planner
	rosrun iscp_path_planner path_planner

5. run demo trajectories
	rosrun iscp_path_planner multi_vehicle_demo_live.py