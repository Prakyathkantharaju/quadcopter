
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