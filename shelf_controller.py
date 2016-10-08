from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
c_hand=0.3
o_hand=0.43
r_hand=0.8
pre_hand=0.75
close_hand=[c_hand,c_hand,c_hand,pre_hand]
open_hand=[o_hand,o_hand,o_hand,pre_hand]
release_hand=[r_hand,r_hand,r_hand,pre_hand]
move_speed=0.9;
face_down_forward=[1,0,0,0,-1,0,0,0,-1]
face_down_backward=[-1,0,0,0,1,0,0,0,-1]
face_down=face_down_forward
start_pos=(face_down,[0.0,0,0.6])
drop_pos=(face_down,[0.5,0,0.6])

class StateMachineController(ReflexController):
	def __init__(self,sim,hand,dt):
		self.sim=sim
		self.hand=hand
		self.robot=sim.world.robot(0)
		self.sim.updateWorld()
		self.state='idle'

	def __call__(self,controller):
		sim=self.sim
		sim.updateWorld()
		current_pos=get_moving_base_xform(self.robot)
		self.go_to(controller,current_pos,start_pos)

	def go_to(self,controller,current_pos,goal_pos):
			t=vectorops.distance(current_pos[1],goal_pos[1])/move_speed
			send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);
	def close_hand(self):
		self.hand.setCommand(close_hand)
	def open_hand(self,hand_config=open_hand):
		self.hand.setCommand(hand_config)


def make(sim,hand,dt):
	return StateMachineController(sim,hand,dt)

