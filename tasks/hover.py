from typing import Optional, Tuple

import dm_control.utils.transformations as tr
import numpy as np
from dm_control import composer
from dm_control.locomotion import arenas
from dm_control.utils import rewards

from scipy.spatial import distance
from sklearn.metrics import mean_squared_error

from tasks. utils import _find_non_contacting_height, _check_hover_status


DEFAULT_CONTROL_TIMESTEP = 0.03
DEFAULT_PHYSICS_TIMESTEP = 0.001

DES_POSITION  = np.array([0,0,1])

#TODO needs more testing
def get_hover_reward(position: float):
    
    pos_reward = np.exp( -100 * mean_squared_error(position, DES_POSITION) )
    # bounds are 0, 1 
    reward = rewards.tolerance(pos_reward,
                               bounds=(0, 1),
                               margin=0,
                               value_at_margin=0,
                               sigmoid='linear')

    return 10 * reward  # [0, 1] => [0, 10]


class Hover(composer.Task):

    def __init__(self,
                 robot,
                 terminate_pitch_roll: Optional[float] = 30,
                 physics_timestep: float = DEFAULT_PHYSICS_TIMESTEP,
                 control_timestep: float = DEFAULT_CONTROL_TIMESTEP,
                 floor_friction: Tuple[float] = (1, 0.005, 0.0001),
                 randomize_ground: bool = True,
                 add_velocity_to_observations: bool = True):

        self.floor_friction = floor_friction
        # just simple floor cause it does not matter.
        self._floor = arenas.Floor(size=(10, 10))

        for geom in self._floor.mjcf_model.find_all('geom'):
            geom.friction = floor_friction

        self._robot = robot
        self._floor.add_free_entity(self._robot)

        # observables = (self._robot.observables.proprioception +
        #                self._robot.observables.kinematic_sensors +
        #                [self._robot.observables.prev_action])

        # NO proprioception yet for the tello env
        observables = (self._robot.observables.kinematic_sensors +
                       [self._robot.observables.prev_action])
        for observable in observables:
            observable.enabled = True

        if not add_velocity_to_observations:
            self._robot.observables.sensors_velocimeter.enabled = False

        if hasattr(self._floor, '_top_camera'):
            self._floor._top_camera.remove()
        self._robot.mjcf_model.worldbody.add('camera',
                                             name='side_camera',
                                             pos=[0, -1, 0.5],
                                             xyaxes=[1, 0, 0, 0, 0.342, 0.940],
                                             mode="trackcom",
                                             fovy=60.0)

        self.set_timesteps(physics_timestep=physics_timestep,
                           control_timestep=control_timestep)

        self._terminate_pitch_roll = terminate_pitch_roll

        self._move_speed = 0.5

    def get_reward(self, physics):
        xmat = physics.bind(self._robot.root_body).xpos
        # _, pitch, _ = tr.rmat_to_euler(xmat, 'XYZ')
        # velocimeter = physics.bind(self._robot.mjcf_model.sensor.velocimeter)

        # gyro = physics.bind(self._robot.mjcf_model.sensor.gyro)

        return get_hover_reward(position=xmat)

    def initialize_episode_mjcf(self, random_state):
        super().initialize_episode_mjcf(random_state)

        # Terrain randomization
        if hasattr(self._floor, 'regenerate'):
            self._floor.regenerate(random_state)
            self._floor.mjcf_model.visual.map.znear = 0.00025
            self._floor.mjcf_model.visual.map.zfar = 50.

        new_friction = (random_state.uniform(low=self.floor_friction[0] - 0.25,
                                             high=self.floor_friction[0] +
                                             0.25), self.floor_friction[1],
                        self.floor_friction[2])
        for geom in self._floor.mjcf_model.find_all('geom'):
            geom.friction = new_friction

    def initialize_episode(self, physics, random_state):
        super().initialize_episode(physics, random_state)
        self._floor.initialize_episode(physics, random_state)

        self._failure_termination = False

        _find_non_contacting_height(physics,
                                    self._robot,
                                    qpos=self._robot._INIT_QPOS)

    def before_step(self, physics, action, random_state):
        pass

    def before_substep(self, physics, action, random_state):
        self._robot.apply_action(physics, action, random_state)

    def action_spec(self, physics):
        return self._robot.action_spec

    def after_step(self, physics, random_state):
        self._failure_termination = False

        if self._terminate_pitch_roll is not None:
            roll, pitch, _ = self._robot.get_roll_pitch_yaw(physics)

            if (np.abs(roll) > self._terminate_pitch_roll
                    or np.abs(pitch) > self._terminate_pitch_roll):
                self._failure_termination = True

        if self._terminate_height:
            xmat = physics.bind(self._robot.root_body).xpos
            hover = _check_hover_status(DES_POSITION, xmat)

            self._failure_termination = not hover


    def should_terminate_episode(self, physics):
        return self._failure_termination

    def get_discount(self, physics):
        if self._failure_termination:
            return 0.0
        else:
            return 1.0

    @property
    def root_entity(self):
        return self._floor
