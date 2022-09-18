# quadcopter
Quadcopter control using RL and low level controller



# Progress and problems
## Progress
- The task and the env are setup.
## Problem
- Observation is qpos it just pos and vel of the body
- But current setup is in the position control but need a force control where action is independent of the joint position.

# Things to do

- Need to setup quadcopter env similar just like A1 env setup here: https://github.com/ikostrikov/walk_in_the_park

- Sensors
   - [x] Add accel and gyro sensors to the drone. (Need to validate the sensor output)
   - [ ] Add the camera to the front of the body ? range of view and other questions for the camera.

- Reward
   -[x] Floating Reward
      This is the reward structure
      - desired_position ($\hat{q}$)= [0,0,1]
      - actual_position ($q$)
      reward = $r_h = exp(-100 * \sum || \hat{q} - q||^2)$
   - [ ] Forward velocity Reward
   - [ ] Trajectory Reward


- Health / env failure
    - [x] Check the Height.
      if the RMSE > (tol*2)**2 in any axis then failure
    - [ ] Check if the trajectory is going in the similar direction.
    
