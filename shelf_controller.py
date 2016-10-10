from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import math
c_hand=0.3
o_hand=0.6
s_hand=0.85
pre_hand=0
close_hand=[c_hand,c_hand,c_hand,pre_hand]
open_hand=[o_hand,o_hand,o_hand,pre_hand]
sweep_hand=[s_hand,s_hand,s_hand,pre_hand]
move_speed=0.9;
face_down=[1,0,0,0,1,0,0,0,1]
# face_toward_shelf=[1,0,0,0,0,1,0,-1,0]
face_toward_shelf=so3.from_axis_angle(([1,0,0],math.pi/180*70))
print face_toward_shelf
start_pos=(face_toward_shelf,[0,0,0.9])
drop_pos=(face_toward_shelf,[0,-0.2,0.79])
sweep_pos_1=(face_down,[-0.05,0.75,0.9])
sweep_pos_2=(face_down,[-0.05,0.75,0.79])
sweep_pos_3=(face_down,[-0.05,0,0.79])
z_min=0.79
z_max=0.93
x_min=-0.1
x_max=0.1
y_max=0.57

test_pos=(face_toward_shelf,[x_min,y_max,z_max])
class StateMachineController(ReflexController):
	def __init__(self,sim,hand,dt):
		self.sim=sim
		self.hand=hand
		self.robot=sim.world.robot(0)
		self.sim.updateWorld()
		self.state='idle'
		self.dt = dt
		self.last_state_end_t=sim.getTime()
		self.target=(so3.identity(),[0,0,0])
		self.num_target=sim.world.numRigidObjects()
		self.current_target=0
		self.waiting_list=range(self.num_target)
		self.score=0
		self.print_flag=0
		self.num_pick=[0]*self.num_target
		self.sweep_try=0
		#get references to the robot's sensors (not properly functioning in 0.6.x)
		self.f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.contact_sensors = self.f1_proximal_takktile_sensors + self.f1_distal_takktile_sensors + self.f2_proximal_takktile_sensors + self.f2_distal_takktile_sensors + self.f3_proximal_takktile_sensors + self.f3_distal_takktile_sensors
		self.verbose = False
		for i in self.waiting_list:
			print sim.world.rigidObject(i).getTransform()[1]
	def __call__(self,controller):
		sim=self.sim
		sim.updateWorld()
		current_pos=get_moving_base_xform(self.robot)
		time=sim.getTime()
		ReflexController.__call__(self,controller)
		if self.print_flag==1:
			print "State:",self.state
			self.print_flag=0
		if self.state=='idle':
			# if time<self.last_state_end_t+2:
			# 	self.go_to(controller,current_pos,start_pos)
			# 	self.open_hand()
			# else:
			# 	self.go_to(controller,current_pos,test_pos)
			if not self.at_destination(current_pos,start_pos) or time<self.last_state_end_t+1:
				self.go_to(controller,current_pos,start_pos)
				self.open_hand(open_hand)
			else:
				print '-----------------------------------------'
				self.check_target()
				print self.num_pick
				if len(self.waiting_list)>0:
					print "Score:",self.score,"/",self.num_target
					self.set_state('find_target')
				elif len(self.waiting_list)<1:
					print "Finished! Total score is",self.score,"/",self.num_target
					print time
					exit()
				min_try=3
				for i in self.waiting_list:
					if self.num_pick[i]<min_try:
						min_try=self.num_pick[i]
				if min_try>0:
					self.set_state('sweep')
				if self.sweep_try==3:
					print "Give up! Total score is",self.score,"/",self.num_target
					print time
					exit()
		elif self.state=='find_target':
			self.find_target()
			self.set_state('pick_target')
		elif self.state=='pick_target':
			if not self.at_destination(current_pos,self.target):
				self.go_to(controller,current_pos,self.target)
			else:
				self.close_hand()
			if time>self.last_state_end_t+2:
				self.set_state('retract')
		elif self.state=='retract':
			if time<self.last_state_end_t+0.5:
				self.go_to_fast(controller,current_pos,drop_pos,0.3)
			elif time<self.last_state_end_t+1:
				self.go_to(controller,current_pos,(face_down,[0,-0.2,0.79]))
				self.open_hand()
			else:
				self.set_state('idle')
				self.go_to(controller,current_pos,start_pos)
		elif self.state=='sweep':
			if time<self.last_state_end_t+2:
				self.open_hand(sweep_hand)
				self.go_to(controller,current_pos,sweep_pos_1)
			elif time<self.last_state_end_t+3:
				self.go_to(controller,current_pos,sweep_pos_2)
			elif time<self.last_state_end_t+4:
				self.go_to_fast(controller,current_pos,sweep_pos_3,0.4)
			else:
				sweep_pos_1[1][0]+=0.04
				sweep_pos_2[1][0]+=0.04
				sweep_pos_3[1][0]+=0.04
				self.set_state('idle')
				self.sweep_try+=1



	def find_target(self):
		self.current_target=self.waiting_list[0]
		for i in range(len(self.waiting_list)):
			if self.num_pick[self.waiting_list[i]]<self.num_pick[self.current_target]:
				self.current_target=self.waiting_list[i]
			elif self.num_pick[self.waiting_list[i]]==self.num_pick[self.current_target]:
				if self.sim.world.rigidObject(self.waiting_list[i]).getTransform()[1][1]<self.sim.world.rigidObject(self.current_target).getTransform()[1][1]:
					self.current_target=self.waiting_list[i]
		p=self.sim.world.rigidObject(self.current_target).getTransform()
		self.target=[face_toward_shelf,vectorops.add(p[1],[0,-0.07,0.06])]
		self.target[1][0]=min(max(x_min,self.target[1][0]),x_max)
		self.target[1][1]=min(y_max,self.target[1][1])
		self.target[1][2]=min(max(z_min,self.target[1][2]),z_max)
		self.num_pick[self.current_target]+=1
		print p[1]
	def check_target(self):
		remove_list=[]
		for i in range(len(self.waiting_list)):
			p=self.sim.world.rigidObject(self.waiting_list[i]).getTransform()[1]
			if p[2]<0.6:
				remove_list.append(self.waiting_list[i])
			if p[0]<0.25 and p[0]>-0.25 and p[1]<0.25 and p[1]>-0.25:
				self.score=self.score+1
		print 'waiting list',self.waiting_list
		print 'remove list',remove_list
		for i in remove_list:
			self.waiting_list.remove(i)
		print 'waiting list',self.waiting_list 
	def go_to(self,controller,current_pos,goal_pos):
			t=vectorops.distance(current_pos[1],goal_pos[1])/move_speed
			send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);
	def close_hand(self):
		self.hand.setCommand(close_hand)
	def open_hand(self,hand_config=open_hand):
		self.hand.setCommand(hand_config)
	def set_state(self,state_name):
			self.state=state_name
			self.last_state_end_t=self.sim.getTime()
			self.print_flag=1
	def at_destination(self,current_pos,goal_pos):
			if vectorops.distance(current_pos[1],goal_pos[1])<0.05:
				return True
			else:	
				return False
	def go_to_fast(self,controller,current_pos,goal_pos,t):
		send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);
		# print "GO TO FAST"
def make(sim,hand,dt):
	return StateMachineController(sim,hand,dt)

