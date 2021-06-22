# coxgraph
A Multi Robot Collaborative Dense Scene Reconstruction

## Installation
- later

## Test
### Experiment CVG
1. Start up reconstruction nodes:

    You can start coxgraph clients and servers in seperate launch files, which are: 
[coxgraph_server.launch](./coxgraph/launch/coxgraph_server.launch),
[coxgraph_client0_cvg.launch](./coxgraph/launch/cvg/coxgraph_client0_cvg.launch) and 
[coxgraph_client1_cvg.launch](./coxgraph/launch/cvg/coxgraph_client1_cvg.launch), 
using these steps:

	1. open multi terminals to run roslaunchs, by tools like `tmux`.
    2. attach to the docker.
    3. in coxgraph catkin workspace:
        ``` 
        source devel/setup.bash
        ```

    4. run each launch file in one of terminals:
        ```
        roslaunch coxgraph coxgraph_server.launch
        roslaunch coxgraph coxgraph_client0_cvg.launch
        roslaunch coxgraph coxgraph_client1_cvg.launch
        ```
    or you can run a wrapped launch: 
    ```
    roslaunch coxgraph run_experiment_cvg.launch
    ```
    in which case, you will get a load of logs in the same terminal, which is hard to debug.
        
2. Start corbslam nodes:

    In another terminal, 
    ```
    cd <path/to/corbslam_ws>
    source devel/setup.bash
    roslaunch corbslam_client corbslam_frontend_cvg.launch
    ```

3. Start rviz:

    In another terminal,
    ```
    cd <path/to/coxgraph_ws>
    source devel/setup.bash
    roscd coxgraph/config
    rviz -d coxgraph.rviz
    ```
4. Debug:

    By adding `debug<_suffix>:=true` arg when starting launch files, an external xterm window will pop up with gdb running. You can refer to launch files, to look for the debug arg suffix for each node.

### Gazebo Simulation Experiment
- WIP

## Service

1. `"coxgraph_server_node/control_trigger"`, see the srv file [ControlTrigger.srv](./coxgraph_msgs/srv/ControlTrigger.srv) for definition.
2. `"coxgraph_server_node/state_query"`, definition at [StateQuery.srv](./coxgraph_msgs/srv/StateQuery.srv)