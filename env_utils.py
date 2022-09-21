from typing import Optional

import gym
import numpy as np
from dm_control import composer
from dmcgym import DMCGYM
from gym import spaces
from gym.wrappers import FlattenObservation

from dm_control import composer

from robots.tello import Tello
from tasks.hover import Hover



def test_env_viewer(task_name: str = 'Hover',
             control_frequency: int = 33,
             randomize_ground: bool = True,
             action_history: int = 1):
    robot = Tello(action_history=action_history)
    # robot.kd = 5

    if task_name == 'Hover':
        task = Hover(robot,
                   control_timestep=round(1.0 / control_frequency, 3),)
    else:
        raise NotImplemented

    env = composer.Environment(task, strip_singleton_obs_buffer_dim=True)

    # env = DMCGYM(env)
    # env = FlattenObservation(env)

    return env

class ClipAction(gym.ActionWrapper):

    def __init__(self, env, min_action, max_action):
        super().__init__(env)

        min_action = np.asarray(min_action)
        max_action = np.asarray(max_action)

        min_action = min_action + np.zeros(env.action_space.shape,
                                           dtype=env.action_space.dtype)

        max_action = max_action + np.zeros(env.action_space.shape,
                                           dtype=env.action_space.dtype)

        min_action = np.maximum(min_action, env.action_space.low)
        max_action = np.minimum(max_action, env.action_space.high)

        self.action_space = spaces.Box(
            low=min_action,
            high=max_action,
            shape=env.action_space.shape,
            dtype=env.action_space.dtype,
        )

    def action(self, action):
        return np.clip(action, self.action_space.low, self.action_space.high)


def make_env(task_name: str = 'Hover',
             control_frequency: int = 33,
             randomize_ground: bool = True,
             action_history: int = 1,
             death: bool = False):
    robot = Tello(action_history=action_history)
    # robot.kd = 5

    if task_name == 'Hover':
        task = Hover(robot,
                   control_timestep=round(1.0 / control_frequency, 3),
                   randomize_ground=randomize_ground, terminate_height=False)
    else:
        raise NotImplemented

    env = composer.Environment(task, strip_singleton_obs_buffer_dim=True)

    env = DMCGYM(env)
    env = FlattenObservation(env)

    return env



make_env.metadata = DMCGYM.metadata


def make_mujoco_env(env_name: str,
                    control_frequency: int,
                    clip_actions: bool = True,
                    action_filter_high_cut: Optional[float] = -1,
                    action_history: int = 1) -> gym.Env:
    env = make_env(env_name,
                   control_frequency=control_frequency,
                   action_history=action_history)

    env = gym.wrappers.TimeLimit(env, 400)

    env = gym.wrappers.ClipAction(env)

    if action_filter_high_cut is not None and False: # Not implemented yet
        env = ActionFilterWrapper(env, highcut=action_filter_high_cut)

    # if clip_actions and False: # Not implemented yet
    #     ACTION_OFFSET = np.asarray([0.2, 0.4, 0.4] * 4)
    #     INIT_QPOS = sim.robots.a1.A1._INIT_QPOS
    #     if env.action_space.shape[0] == 12:
    #         env = ClipAction(env, INIT_QPOS - ACTION_OFFSET,
    #                          INIT_QPOS + ACTION_OFFSET)
    #     else:
    #         env = ClipAction(
    #             env, np.concatenate([INIT_QPOS - ACTION_OFFSET, [-1.0]]),
    #             np.concatenate([INIT_QPOS + ACTION_OFFSET, [1.0]]))
    env = ClipAction(env, np.array([0,0,0,0]), np.array([1,1,1,1]) )

    return env
