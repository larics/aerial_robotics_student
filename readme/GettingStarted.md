# Getting started
These instructions will provide how to start and run the simulation.

## Running the simulation
When docker has been started with `./start_docker.sh` it should open in `~/startup` folder. To start the simulation simply run
```bash
./start.sh
```

The nodes that will be running are described in `session.yml`. This procedure uses a `tmux` extension called `tmuxinator`. Instead of running each process in each pane separately, you can specify a config file, in this case `session.yml`, with all that needs to be run.

The simulation should start, alongside `tmux` you should see `Gazebo`, `Rviz` and `rqt` gui perspective being run. There are multiple windows in tmux:
1. `crazyflies_start` - Crazyflies simulation
2. `util` - Rqt gui, control input mux, rviz
3. `height_control` - Height control node
4. `yaw_control` - Yaw control node
5. `horizontal_control` - Horizontal control node

**Note**: Each `control` node publishes its own control input separately. `mux.py` is responsible for merging these control values and sending them to the appropriate crazyflies topic.

### Remark on running
Each time you run a simulation, `control` nodes might not start properly. This is intended behavior. As you start with your tasks you will implement various control components and the given control node will run once it is properly implemented.

## Using rqt gui for tuning
For each task, a separate `rqt` perspective is created for tuning. You are free to use the given perspectives as they enable easier tuning of the controllers.

To load a perspective, go to `Perspectives->Import`. The prepared perspectives can be found in `~/ros2_ws/src/aerial_robotics/aerial_robotics_tasks/config`.

Each perspective will have three separate parts:
1. **Paremeter Reconfigure** - You can change PID parameters for each control node here.
2. **Message Publisher** - Prepared several step references to publish and tune.
3. **PyQtGraph** - Plot visualization of relevant values helpful for tuning the controller.

One feature that can also be utilized is export from `PyQtGraph`. Simply right-click on the graph you want to export and click `Export`. We recommend to export `CSV` file and stylize the plot in `Matlab` or some other program suitable for plotting.

## Other ROS tools
Feel free to explore using `ros2 topic list`, `ros2 topic echo`, `ros2 bag` and similar.