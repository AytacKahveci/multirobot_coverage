# Multi-robot Coverage Path Planning Problem

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/l0PaeuPWRHk/0.jpg)](https://www.youtube.com/watch?v=l0PaeuPWRHk)

## Repository

- fsm.py: Finite State Machine implementation for distributed algorithm
- coverage_plan.py: Voronoi partition algorithm
- path_plan.py: Coverage path planning algorithm
- fuzzy_controller.py: Fuzzy controller implementation for path following
- robot_tf.py: Simple diff_drive robot simulation

## Region partition with Voroni
Algorithm is implemented in the **coverage_plan.py** script. \
Academic paper for this algorithm can be found in [here](https://www.researchgate.net/publication/329624395_VORONOI_MULTI-ROBOT_COVERAGE_CONTROL_IN_NON-CONVEX_ENVIRONMENTS_WITH_HUMAN_INTERACTION_IN_VIRTUAL_REALITY) \
Repo which is utilized for this algorithm can be found in [here](https://github.com/lucascoelhof/voronoi_hsi)

## Coverage Path planning
Algorithm is implemented in the *path_plan.py* script.
Academic paper for this algorithm can be found in [here](https://www.researchgate.net/publication/257518297_BA_An_online_complete_coverage_algorithm_for_cleaning_robots)


## Requirements
- pytransitions \
  ```pip install transitions```

- opencv

- scikit-fuzzy \
  ```pip install -U scikit-fuzzy```

## Test
  Test with robot sim:
  ```bash
  # Launch map
  roslaunch multirobot_coverage navigation_map.launch

  # Launch finite state machine
  roslaunch multirobot_coverage test.launch

  # Launch robot sim which simulates path following action of the robot
  roslaunch multirobot_coverage robot_sim.launch

  # To start finite state machine
  rostopic pub -1 /robot1/start_request std_msgs/Bool "data: true"
  ```


  Test without robot sim:
  ```bash
  # Creates two robots and partitions the map then calculates coverage path of a robot
  # Outputs to /planned_path topic as coverage path
  python main.py
  ```