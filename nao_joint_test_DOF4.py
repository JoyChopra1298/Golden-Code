
import mujoco_py
import copy
import numpy as np
from gym.envs.robotics import rotations, robot_env, utils
import time
model = mujoco_py.load_model_from_path("envs/assets/nao_hand.xml")

n_substeps=10
sim = mujoco_py.MjSim(model,nsubsteps=n_substeps) 
data = sim.data

initial_state=copy.deepcopy(sim.get_state())
sim.forward()
viewer = mujoco_py.MjViewer(sim)
viewer.cam.trackbodyid = 20
def rotate(sim,num_steps):
	joint_name_list=sim.model.joint_names
	print (joint_name_list)
	a=["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll"]
	valqpos=list(map(sim.data.get_joint_qpos,a))
	print(valqpos)
	for j in range(num_steps):
		viewer.render()
		sim.data.set_joint_qpos(a[3], valqpos[3]+0.05*j)
		sim.forward()
		sim.step()

rotate(sim,1900)
