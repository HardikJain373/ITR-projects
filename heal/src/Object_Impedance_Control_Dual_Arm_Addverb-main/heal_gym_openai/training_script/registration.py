'''
# Author : Rajesh Kumar (rajesh.kumar01@addverb.com)
# Perform all the initializations 
'''

import os 
import sys 
import gymnasium as gym 
import numpy as np

sys.path.append("..")

from env.heal_env import HealEnv

from gymnasium.envs.registration import register

register(
    id="Heal-v1",
    # path to the class for creating the env
    # Note: entry_point also accept a class as input (and not only a string)
    entry_point="env.heal_env:HealEnv",
    # Max number of steps per episode, using a `TimeLimitWrapper`
    max_episode_steps=500,
)