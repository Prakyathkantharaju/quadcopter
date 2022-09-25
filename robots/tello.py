import os
from collections import deque
from functools import cached_property
from typing import Optional

import numpy as np
from dm_control import composer, mjcf
from dm_control.composer.observation import observable
from dm_control.locomotion.walkers import base
from dm_control.utils.transformations import quat_to_euler
from dm_env import specs
# This file is very similar to the https://github.com/ikostrikov/walk_in_the_park/blob/main/sim/robots/a1.py
ASSETS_DIR = os.path.join(os.path.dirname(__file__), 'assets')
_TELLO_XML_PATH = os.path.join(ASSETS_DIR, 'xml', 'quadrotor_hovering.xml')

# TODO change this
class TelloObservables(base.WalkerObservables):

    @composer.observable
    def joints_vel(self):
        return observable.MJCFFeature('qvel', self._entity.observable_joints)

    @composer.observable
    def prev_action(self):
        return observable.Generic(lambda _: self._entity.prev_action)

    @property
    def proprioception(self):
        return ([self.joints_pos, self.joints_vel] +
                self._collect_from_attachments('proprioception'))

    @composer.observable
    def sensors_velocimeter(self):
        return observable.Generic(
            lambda physics: self._entity.get_velocity(physics))

    # @composer.observable
    # def base_position(self):
    #     print(self._entity.root_body.xpos)
    #     return observable.Generic( lambda _: self._entity.root_body.xpos)

    @composer.observable
    def base_position(self):
        return observable.Generic( lambda physics : self._entity.get_position(physics))

    @composer.observable
    def base_orientation(self):
        return observable.Generic( lambda physics : self._entity.get_orientation(physics))

    @property
    def kinematic_sensors(self):
        return ([
            self.sensors_gyro, self.sensors_accelerometer
        ])


class Tello(base.Walker):
    _INIT_QPOS = np.asarray([0, 0, 1]) # joint posiition at the start of the system
    _QPOS_OFFSET = np.asarray([0.2, 0.4, 0.4] * 4) # I am not sure what is this used for ?
    # _INIT_QPOS = np.asarray([-0.1, 0.5, -1.4] * 4)
    """A composer entity representing a Jaco arm."""

    def _build(self,
               name: Optional[str] = None,
               action_history: int = 1,
               learn_kd: bool = False):
        """Initializes the JacoArm.

    Args:
      name: String, the name of this robot. Used as a prefix in the MJCF name
        name attributes.
    """
        self._mjcf_root = mjcf.from_path(_TELLO_XML_PATH)
        if name:
            self._mjcf_root.model = name
        # Find MJCF elements that will be exposed as attributes.
        self._root_body = self._mjcf_root.find('body', 'quadrotor')
        # self._root_body.pos[-1] = 0.125

        self._joints = self._mjcf_root.find_all('joint')

        self._actuators = self.mjcf_model.find_all('actuator')


        # Check that joints and actuators match each other.
        assert len(self._joints) == 0
        assert len(self._actuators) == 4
        #TODO why is this needed
        # for joint, actuator in zip(self._joints, self._actuators):
        #     assert joint == actuator.joint

        self.kp = 60
        if learn_kd:
            self.kd = None
        else:
            self.kd = 10

        self._prev_actions = deque(maxlen=action_history)
        self.initialize_episode_mjcf(None)

    def initialize_episode_mjcf(self, random_state):
        self._prev_actions.clear()
        for _ in range(self._prev_actions.maxlen):
            self._prev_actions.append(self._INIT_QPOS)

    @cached_property
    def action_spec(self):
        minimum = []
        maximum = []

        for actuator in self.actuators:

            minimum.append(actuator.ctrlrange[0])
            maximum.append(actuator.ctrlrange[1])

        if self.kd is None:
            minimum.append(-1.0)
            maximum.append(1.0)
        

        return specs.BoundedArray(
            shape=(len(minimum), ),
            dtype=np.float32,
            minimum=minimum,
            maximum=maximum,
            name='\t'.join([actuator.name for actuator in self.actuators]))

    @cached_property
    def ctrllimits(self):
        minimum = []
        maximum = []
        for actuator in self.actuators:
            minimum.append(actuator.ctrlrange[0])
            maximum.append(actuator.ctrlrange[1])

        return minimum, maximum

    def apply_action(self, physics, actions, random_state):
        # Updates previous action.
        self._prev_actions.append(actions.copy())
        minimum, maximum = self.ctrllimits
        action = np.clip(actions, minimum, maximum)

        physics.bind(self.actuators).ctrl = action

    def _build_observables(self):
        return TelloObservables(self)

    @property
    def root_body(self):
        return self._root_body

    @property
    def joints(self):
        """List of joint elements belonging to the arm."""
        return self._joints

    @property
    def observable_joints(self):
        return self._joints

    @property
    def actuators(self):
        """List of actuator elements belonging to the arm."""
        return self._actuators

    @property
    def mjcf_model(self):
        """Returns the `mjcf.RootElement` object corresponding to this robot."""
        return self._mjcf_root

    @property
    def prev_action(self):
        return np.concatenate(self._prev_actions)

    def get_roll_pitch_yaw(self, physics):
        quat = physics.bind(self.mjcf_model.sensor.framequat).sensordata
        return np.rad2deg(quat_to_euler(quat))

    def get_velocity(self, physics):
        velocimeter = physics.bind(self.mjcf_model.sensor.velocimeter)
        return velocimeter.sensordata

    def get_position(self, physics):
        pos = physics.bind(self._root_body).xpos
        return pos

    def get_orientation(self, physics):
        quat = physics.bind(self._root_body).xquat
        euler = quat_to_euler(quat)
        return euler