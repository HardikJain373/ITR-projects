<!-- Author : Rajesh Kumar (rajesh.kumar01@addverb.com) 
Description : This is the OPENAI Baselines gym examples for Heal Robot; tested on a virtual system with the following -->

# LICENSE 
License to use the program 

# Spec File 
spec-file.txt : Conda spec file on which programs were tested : cuda versions to be changed; specific details are : 

# SETUP THE SYSTEM
Check the versions in 
spec-file.yml

To check the versions and manually setup the environment : check version.txt

# Examples 
SAC example; single sim :  

```bash
cd training_script
python sac.py
```

For multi sim in vectorized form : 
```bash
cd training_script
python ppo.py
```

# Environment Parameters 

Check config.ini file; the model is present in env/model/heal.xml 

User can change contact elements from contact tag and change gains for velocity control and move to torque control 

# Write a new training program 

Edit the  HealEnv Class and implement the following functions : 
1. is_done : To check whether the goal is achieved 
2. _is_truncated : Restart function 
3. get_reward : Reward function 
4. _get_info : Dict to get the info 


# Using the dockerfile 

In order to use the dockerfile; the local Display environment has to forwarded to the local host variable. In the local terminal; ensure clients can connect from any host; type 

```bash
xhost +local 
```

Now, note the local display environment variable 
```bash 
# See the display id : to use the same id within the docker terminal 
echo "DISPLAY = $DISPLAY"
```

Now, build the dockerfile; avoid the step if its already built : 

```bash 
# build the docker container 
sudo docker build -t ${TAG} .
```

Run the docker container 

```bash
sudo docker container run --rm --net host -v /tmp/.X11-unix:/tmp/.X11-unix -it ${TAG}
```

Now, set the Display variable as the host variable within the docker container : 

```bash
export DISPLAY=:${HOST_DISPLAY_ID}
```

Now, run the file as : 

```bash
python3 sac.py
```





