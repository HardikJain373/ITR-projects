'''
# Author : Rajesh Kumar (rajesh.kumar01@addverb.com)
# Details : This is the OpenAI SAC example for the heal robot 
'''

import registration
import gymnasium as gym 

from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC 


env = gym.make("Heal-v1", render_mode="human") 

# default device is cuda 
model = SAC("MlpPolicy", env, verbose = 1)

model.learn(total_timesteps=50000, log_interval=10)

model.save("heal_cobot")

env = model.get_env()
obs = env.reset()

while True:
    action, _states = model.predict(obs)

    obs, rewards, dones, info = env.step(action)
    env.render()