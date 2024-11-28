'''
    # Author : Rajesh Kumar (rajesh.kumar01@addverb.com)
    # Details : This is the OpenAI PPO example for the heal robot with multiple vectorized environments 
'''

import gymnasium as gym 

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import registration

# Parallel environments
# N envs = no of environments to be created 
vec_env = make_vec_env("Heal-v1", n_envs=4)

model = PPO("MlpPolicy", vec_env, verbose=1, device="cuda")
model.learn(total_timesteps=50000)
model.save("Heal")


model = PPO.load("Heal")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
    # print(rewards)
    print(action)
    vec_env.render("human")