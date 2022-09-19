# quadcopter
Quadcopter control using RL and low level controller



# Overview of the setup
!()[docs/images/quadcopter.png]

### Observations
- global position
- gyroscope
- accelerometer

### action
- Action array of size (4) controlling motor.
- range [0,1]

## Robot setup
- Setting up the Tello robot in robots.
- XML file in this [assets/xml](robots/assets/xml)
- The robot will give observation and the action setup.

## Task
- Setting up the reward function, stoppage etc. 
- Currently there is only one task `hover` like to task file is [here](tasks/hover.py)

## Test scripts
- `test_obs.py` To check the observation, reward, done and reward for each setup. This is done using dm_control setup. To run this.
``` bash
python tests/test_obs.py
```
- `test_viewer.py` To run the viewer. Using the dm_control setup. To run this.
``` bash
python tests/test_viewer.py
```
!()[docs/images/viewer_setup.gif]

# Progress and problems
## Progress
- The task and the env are setup.
- Observation is qpos it just pos and vel of the body

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
    
