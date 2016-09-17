from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import time
c_hand=0.35
o_hand=0.4
pre_hand=0.7
close_hand=[c_hand,c_hand,c_hand,pre_hand]
open_hand=[o_hand,o_hand,o_hand+0.1,pre_hand]
move_speed=0.5;
face_down=[1,0,0,0,-1,0,0,0,-1]
start_pos=(face_down,[0,0,0.5])
drop_pos=(face_down,[0.7,0,0.5])


class StateMachineController(ReflexController):
	"""A more sophisticated controller that uses a state machine."""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.robot=sim.world.robot(0)
		self.dt = dt
		self.sim.updateWorld()
		self.base_xform = get_moving_base_xform(self.sim.controller(0).model())
		self.state = 'idle'
		self.last_state_end_t=sim.getTime()
		self.target=(so3.identity(),[0,0,0])
		self.num_ball=sim.world.numRigidObjects()
		self.current_target=0
		self.waiting_list=range(self.num_ball)
		self.score=0
		self.print_flag=0
	def __call__(self,controller):
		sim = self.sim
		sim.updateWorld()
		# xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = False
		ReflexController.__call__(self,controller)
		time=sim.getTime();
		current_pos=get_moving_base_xform(self.sim.world.robot(0))
		#controller state machine
		if self.print_flag==1:
			print "State:",self.state
			self.print_flag=0
		if self.state == 'idle':
			self.go_to(controller,current_pos,start_pos)
			self.open_hand()
			if time>self.last_state_end_t+2 and len(self.waiting_list)>0:
				self.state='find_target'
				self.last_state_end_t=time
				self.print_flag=1
			elif len(self.waiting_list)<1:
				print "Finished! Total score is",self.score,"/",self.num_ball
		elif self.state=='find_target':
			self.target=self.find_target();
			if time>self.last_state_end_t+0.5:
				self.state='pick_target'
				self.last_state_end_t=time
				self.print_flag=1
		elif self.state == 'pick_target':
			if time<self.last_state_end_t+1:
				self.go_to(controller,current_pos,(self.target[0],vectorops.add(self.target[1],[0,0,0.2])))
			elif time<self.last_state_end_t+3:
				self.go_to(controller,current_pos,self.target)
			elif time<self.last_state_end_t+4:
				#this is needed to stop at the current position in case there's some residual velocity
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				self.close_hand()
			else:
				self.state='raising'
				self.last_state_end_t=time
				self.print_flag=1
		elif self.state=='raising':
			raise_pos=(current_pos[0],[current_pos[1][0],current_pos[1][1],0.7])
			if time<self.last_state_end_t+1.5 :
				self.go_to(controller,current_pos,raise_pos)
			else:
				self.state='move_to_drop_position'
				self.last_state_end_t=time
				self.print_flag=1
		elif self.state == 'move_to_drop_position':
			if time<self.last_state_end_t+3 :
				self.go_to(controller,current_pos,drop_pos)
			else:
				self.state='drop'
				self.last_state_end_t=time
				self.print_flag=1
		elif self.state=='drop':
			if time<self.last_state_end_t+0.5 :
				self.open_hand()
			else:
				self.state='idle'
				self.last_state_end_t=time
				self.print_flag=1
				self.check_target()
	def at_destination(self,current_pos,goal_pos):
		if se3.distance(current_pos,goal_pos)<0.05:
			return True
		else:	
			return False
	def find_target(self):
		self.current_target=self.waiting_list[0]
		best_p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		for i in self.waiting_list:
			p=self.sim.world.rigidObject(i).getTransform()[1]
			if p[2]>best_p[2]:
				self.current_target=i
				best_p=p
			elif vectorops.distance([0,0],[p[0],p[1]])<vectorops.distance([0,0],[best_p[0],best_p[1]]):
				self.current_target=i
				best_p=p
		d_x=0
		if best_p[0]>0.15:
			d_x=-0.05
			print 'too close to the wall!!'
		elif best_p[0]<-0.15:
			d_x=0.05
			print 'too close to the wall!!'
		target=(face_down,vectorops.add(best_p,[d_x,0,0.16]))
		return target
	def go_to(self,controller,current_pos,goal_pos):
		t=vectorops.distance(current_pos[1],goal_pos[1])/move_speed
		send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);
	def close_hand(self):
		self.hand.setCommand(close_hand)
	def open_hand(self):
		self.hand.setCommand(open_hand)
	def check_target(self):
		p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		if p[0]>0.25 or p[0]<-0.25 or p[1]>0.25 or p[1]<-0.25:
			self.waiting_list.remove(self.current_target)
		if p[0]<0.95 and p[0]>0.45 and p[1]<0.25 and p[1]>-0.25:
			self.score=self.score+1
		
def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	return StateMachineController(sim,hand,dt)
