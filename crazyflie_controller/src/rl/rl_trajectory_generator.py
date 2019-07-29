#!/usr/bin/env python

import numpy as np
# import pandas as pd
import quaternion
import sys
import os

from quadrotor_environment.utils import rotate_vector


from quadrotor_environment.observation_history import ObservationHistory
from quadrotor_environment.observation_mapping import ToObservationMap
from quadrotor_environment.propeller_speed_mapping import PropellerSpeedMapping
from quadrotor_environment.quadrotor_model import SysState, QuadrotorModel


class StandaloneAgent:
    def __init__(self, observation_mapping_parameters,
                 observation_history_parameters,
                 observe_prop_rate,
                 propeller_speed_mapping_parameters,
                 quadrotor_model_parameters,
                 evaluate_network,
                 pd_support_parameters = None):
        if pd_support_parameters is not None:
            raise ValueError("PD Supported Standalone agents are not supported")

        self.quadrotor_model = quadrotor_model = QuadrotorModel(**quadrotor_model_parameters)

        if observe_prop_rate:
            self.state_to_observation = ToObservationMap(
                propeller_speed_scale=0.1 / quadrotor_model.hovering_thrust,
                **observation_mapping_parameters)
        else:
            self.state_to_observation = ToObservationMap(**observation_mapping_parameters)

        self.observation_history = ObservationHistory(**observation_history_parameters)

        self.action_to_propeller_speed = PropellerSpeedMapping(quadrotor_model.hovering_thrust,
                                                               **propeller_speed_mapping_parameters)

        self.last_relative_propeller_speed = np.ones(4) * quadrotor_model.hovering_thrust

        self.evaluate_network = evaluate_network

    def compute_rotor_rate(self, state: SysState, target_position, observation_age = 0, dt=0.01) -> np.ndarray:
        # Compute observation
        self.state_to_observation.position_offset = target_position
        observation = self.observation_history.get_observation_with_history(
            self.state_to_observation(state, dt, observation_age),
            self.last_relative_propeller_speed, dt)

        # Compute action from network
        action = self.evaluate_network(observation)

        # Compute actual propeller speeds from network action
        propeller_speeds, self.last_relative_propeller_speed, relative_propeller_acceleration = self.action_to_propeller_speed(
            action, self.observation_history.dt_history[-1])  # This is a hacky way to access the most recent dt

        return propeller_speeds


class LinearNumpyModel:
    __slots__ = 'w', 'b'

    def __init__(self, parameter_file_name):
        print(os.path.abspath(parameter_file_name))
        weights = np.load(parameter_file_name)
        self.w = weights['w']
        self.b = weights['b']

    def __call__(self, observation):
        return np.clip(np.matmul(observation, self.w) + self.b, -1, 1)


def generate_trajectory(agent: StandaloneAgent, initial_state: SysState, target: np.ndarray, dt: float, t_s: float, initial_propeller_speed: np.ndarray=None):
    if initial_propeller_speed is None:
        initial_propeller_speed = np.ones(4) * agent.quadrotor_model.hovering_thrust

    initial_observation = agent.state_to_observation(initial_state, dt, 0)
    agent.observation_history.reset(initial_observation,
                                    initial_propeller_speed * 0.1 / agent.quadrotor_model.hovering_thrust,
                                    dt)
    agent.last_relative_propeller_speed = agent.action_to_propeller_speed.reset(initial_propeller_speed)

    t = 0
    x = initial_state

    while t <= t_s + 1e-7:
        u = agent.compute_rotor_rate(x, target, dt=dt)
        yield x, u, t
        x = agent.quadrotor_model.next_state(x, u, dt)
        t += dt

def make_trajectory_generator():
    observation_mapping_parameters = {'angular_velocity_scale': 0.15,
                                      'dt_scale': 0,
                                      'huber_scaling': False,
                                      'local_observations': True,
                                      'observation_age_scale': 0,
                                      'observe_rotation': True,
                                      'position_scale': 0.5,
                                      'rotation_observation_mode': 'rotmat_global',
                                      'velocity_scale': 0.5}
    propeller_speed_mapping_parameters = {'acceleration_constrain_mode': 'tanh',
                                          'output_control_mode': 'direct',
                                          'relative_max_propeller_acceleration': 1518.787631599199,
                                          'relative_max_speed': 3,
                                          'relative_min_speed': 0}
    observation_history_parameters = {'differential_action_observation_history': False,
                                      'differential_state_observation_history': False,
                                      'history_scale': 1,
                                      'num_observed_past_actions': 0,
                                      'num_observed_past_states': 1}
    quadrotor_model_parameters = {'I': [0.007, 0.007, 0.012],
                                  'b': 0.016,
                                  'g0': 9.80665,
                                  'ip': 0,
                                  'kp': 1,
                                  'lp': 0.17,
                                  'm': 0.665}
    agent = StandaloneAgent(observation_history_parameters=observation_history_parameters, observe_prop_rate=False,
                            observation_mapping_parameters=observation_mapping_parameters,
                            propeller_speed_mapping_parameters=propeller_speed_mapping_parameters,
                            quadrotor_model_parameters=quadrotor_model_parameters,
                            evaluate_network=LinearNumpyModel("parameters.npz"))
    def gen_traj(
        position = [1, 0, 0], 
        velocity = [0, 0, 0], 
        rotation_quat = [1, 0, 0, 0], 
        angular_velocity = [0, 0, 0], 
        target: np.ndarray = [0, 0, 0], 
        dt: float = 0.02, 
        t_s: float = 1, 
        initial_propeller_speed: np.ndarray=None):
        initial_state = SysState(np.asarray(position),
                                 np.asarray(velocity),
                                 np.quaternion(*rotation_quat),
                                 np.asarray(angular_velocity),
                                 np.ones(4) * agent.quadrotor_model.hovering_thrust)

        trajectory = []
        for state, u, t in generate_trajectory(agent, initial_state, target, dt, t_s):
            trajectory.append(np.concatenate([state.position, state.velocity, state.rotation.components, state.angular_velocity, u, [t]]))
        return np.asarray(trajectory)

    return gen_traj

if __name__ == '__main__':
    traj_gen = make_trajectory_generator()
    import time

    start = time.time()
    for i in range(100):
        traj_gen()
    stop = time.time()

    print((stop - start) * 1000 / 100)
        # start = np.random.rand(3) * 2 - 1
        # trajectory = traj_gen(start, [0, 0, 0], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0], 0.01, 2)
        # np.savetxt("traj_{}.txt".format(i), trajectory)
            # fh.write("\n".join("{} {} {}".format(*p) for p in trajectory['position']))
            # fh.write("\n\n\n")


