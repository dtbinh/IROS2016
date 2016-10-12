from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *
from reflex_control import *
import math
#two finger configurations
c_hand=0.28
o_hand=0.6
power_hand=0
precision_hand=0.75
power_close_hand=[c_hand,c_hand,c_hand,power_hand]
power_open_hand=[o_hand-0.02,o_hand-0.02,o_hand-0.02,power_hand]
precision_close_hand=[c_hand,c_hand,c_hand,precision_hand]
precision_open_hand=[o_hand,o_hand,o_hand,precision_hand]
#hand orientation
move_speed=0.5;
face_down=[0,-1,0,1,0,0,0,0,1]
face_down_rot=[-1,0,0,0,-1,0,0,0,1]
# face_toward_shelf=[1,0,0,0,0,1,0,-1,0]
face_toward_shelf=so3.mul(so3.from_axis_angle(([1,0,0],math.pi/180*70)),face_down)
idle_pos=(face_down,[0,0.5,0.77])

start_pos_face_down=(face_down,[0,0.4,0.77])
start_pos_face_down_rot=(face_down_rot,[0,0.4,0.77])
start_pos_face_toward_shelf=(face_toward_shelf,[0,0.4,0.77])
drop_pos_face_down=(face_down,[0,-0.1,0.77])
drop_pos_face_down_rot=(face_down_rot,[0,-0.1,0.77])
drop_pos_face_toward_shelf=(face_toward_shelf,[0,-0.2,0.77])

sweep_pos=so3.mul(so3.from_axis_angle(([1,0,0],math.pi/180*70)),face_down_rot)
start_pos_sweep=(sweep_pos,[0,0.55,0.77])
drop_pos_sweep=(sweep_pos,[0,-0.1,0.77])
# sweep_pos_1=(face_down,[-0.03,0.85,0.7])
# sweep_pos_2=(face_down,[-0.03,0.85,0.59])
# sweep_pos_3=(face_down,[-0.03,0,0.59])
#hand position limit
forward_min=[-0.05,-5,0.6]
forward_max=[0.05,0.85,0.75]
face_down_min=[-0.07,-5,0.7]
face_down_max=[0.05,0.9,0.78]


test_pos=(face_toward_shelf,[0,0.7,0.71])
class StateMachineController(ReflexController):
	def __init__(self,sim,hand,dt):
		self.sim=sim
		self.hand=hand
		self.robot=sim.world.robot(0)
		self.sim.updateWorld()
		self.state='idle'
		self.dt = dt
		self.last_state_end_t=sim.getTime()
		self.target_pos=(so3.identity(),[0,0,0])
		self.target_prep_pos=(so3.identity(),[0,0,0])
		self.num_target=sim.world.numRigidObjects()
		self.target_id=0
		self.waiting_list=range(self.num_target)
		self.score=0
		self.print_flag=0
		self.num_pick=[0]*self.num_target
		self.hand_mode='power'
		self.hand_facing='face_down'
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
			print 'object',i
			print sim.world.rigidObject(i).getTransform()[1]
			print sim.world.rigidObject(i).geometry().getBB()
	def __call__(self,controller):
		sim=self.sim
		sim.updateWorld()
		current_pos=get_moving_base_xform(self.robot)
		time=sim.getTime()
		ReflexController.__call__(self,controller)
		controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
		if self.print_flag==1:
			print "State:",self.state
			self.print_flag=0
		if self.state=='idle':
			min_try=5
			for i in self.waiting_list:
				if self.num_pick[i]<min_try:
					min_try=self.num_pick[i]
			if min_try>5:
				print 'That''s it! Score:',self.score,'/',self.num_target
				print time
				exit()
			if time<self.last_state_end_t+2:
				if self.hand_facing=='face_down':
					self.go_to(controller,current_pos,start_pos_face_down)
					self.close_hand()
				elif self.hand_facing=='face_toward_shelf':
					self.go_to(controller,current_pos,start_pos_face_toward_shelf)
					self.close_hand()
				else:
					self.go_to(controller,current_pos,start_pos_sweep)
					self.close_hand()
			else:
				self.check_target()
				if len(self.waiting_list)<1:
					print 'finished! Score:',self.score,'/',self.num_target
					print time
					exit()
				self.set_state('find_target')
		elif self.state=='find_target':
			self.find_target()
			print 'current waiting list:',self.waiting_list
			print 'num of pick:',self.num_pick
			print 'target is:',self.target_id
			self.set_state('start_position')
		elif self.state=='start_position':
			if time<self.last_state_end_t+2:
				if self.hand_facing=='face_down':
					self.go_to(controller,current_pos,start_pos_face_down)
					self.open_hand()
				elif self.hand_facing=='face_toward_shelf':
					self.go_to(controller,current_pos,start_pos_face_toward_shelf)
					self.open_hand()
				else:
					self.go_to(controller,current_pos,start_pos_sweep)
					self.open_hand()
			else:
				self.set_state('pick_position')
		elif self.state=='pick_position':
			if self.hand_facing!='sweep':
				if time<self.last_state_end_t+1:
					self.go_to(controller,current_pos,self.target_prep_pos)
				elif time<self.last_state_end_t+3:
					self.go_to(controller,current_pos,self.target_pos)
				elif time<self.last_state_end_t+3.5:
					self.close_hand()
				elif time<self.last_state_end_t+4:
					self.go_to(controller,current_pos,self.target_prep_pos)
				else:
					self.set_state('retract')
			else:
				if time<self.last_state_end_t+1:
					self.go_to(controller,current_pos,self.target_prep_pos)
				elif time<self.last_state_end_t+3:
					self.go_to(controller,current_pos,self.target_pos)
				elif time<self.last_state_end_t+4:
					# temp_pos=self.target_pos
					# temp_pos[1][1]=self.target_pos[1][1]+0.01
					# self.go_to(controller,current_pos,temp_pos)
					self.close_hand()
				elif time<self.last_state_end_t+5:
					self.go_to(controller,current_pos,self.target_prep_pos)
				else:
					self.set_state('retract')
		elif self.state=='retract':
			if not self.item_in_hand(current_pos):
				self.set_state('idle')
			if self.hand_facing=='face_down':
				drop_pos=drop_pos_face_down
			elif self.hand_facing=='face_toward_shelf':
				drop_pos=drop_pos_face_toward_shelf
			else:
				drop_pos=drop_pos_sweep
			self.go_to(controller,current_pos,drop_pos)
			if self.at_destination(current_pos,drop_pos):
				self.set_state('drop')
		
		elif self.state=='drop':
			if time<self.last_state_end_t+1.5:
				self.open_hand()
			else:
				self.set_state('idle')
			
	



	def item_in_hand(self,current_pos):
		min_dist=10
		for i in self.waiting_list:
			p=self.sim.world.rigidObject(i).getTransform()[1]
			if vectorops.distance(current_pos[1],p)<min_dist:
				min_dist=vectorops.distance(current_pos[1],p)
		if min_dist<0.4:
			return True
		else:
			return False
	def find_target(self):
		self.target_prep_pos=(so3.identity(),[0,10,-1])
		count=0
		# print self.waiting_list
		for idx in self.waiting_list:
			# print 'i',idx
			bb=self.sim.world.rigidObject(idx).geometry().getBB()
			#decide the hand facing and finger configuration
			z_limit=0.65
			ratio=1.0
			if bb[1][2]>z_limit:
				hand_facing='face_toward_shelf'
				count+=1
			else:
				hand_facing='face_down'
				count+=1
			if bb[1][2]<0.54 and self.hand_facing!='sweep':
				hand_facing='sweep'

			if hand_facing=='face_toward_shelf':
				max_limit=forward_max
				min_limit=forward_min
				face=face_toward_shelf
				hand_mode='power'
			elif hand_facing=='face_down':
				max_limit=face_down_max
				min_limit=face_down_min
				x_diff=bb[1][0]-bb[0][0]
				y_diff=bb[1][1]-bb[0][1]
				if y_diff>x_diff*ratio:
					hand_mode='power'
					face=face_down
				elif x_diff>y_diff*ratio:
					hand_mode='power'
					face=face_down_rot
				else:
					hand_mode='precision'
					face=face_down
			else:
				face=sweep_pos
				hand_mode='power'
				
			if hand_facing=='face_down':
				target_prep_pos=(face,vectorops.add(vectorops.div(vectorops.add(bb[1],bb[0]),2),[0,0,0.5]))
				top_bb=bb[1][2]
				target_pos=(face,vectorops.div(vectorops.add(bb[1],bb[0]),2))
				target_pos[1][2]=top_bb+0.06
				for i in range(3):
					target_prep_pos[1][i]=max(min(target_prep_pos[1][i],max_limit[i]),min_limit[i])
					target_pos[1][i]=max(min(target_pos[1][i],max_limit[i]),min_limit[i])
			elif hand_facing=='face_toward_shelf':
				target_prep_pos=(face,vectorops.add(vectorops.div(vectorops.add(bb[1],bb[0]),2),[0,0,0.02]))
				back_bb=bb[0][1]
				target_pos=(face,vectorops.div(vectorops.add(bb[1],bb[0]),2))
				target_prep_pos[1][1]=back_bb-0.07
				target_pos[1][1]=back_bb-0.04
				for i in range(3):
					target_prep_pos[1][i]=max(min(target_prep_pos[1][i],max_limit[i]),min_limit[i])
					target_pos[1][i]=max(min(target_pos[1][i],max_limit[i]),min_limit[i])
			else:
				target_prep_pos=(face,[(bb[1][0]+bb[0][0])*0.5,0.55,0.66])
				y_len=bb[1][1]-bb[0][1]
				target_pos=(face,[(bb[1][0]+bb[0][0])*0.5,0.92-y_len,0.66])
			if count>0 and hand_facing=='sweep':
				continue
			if target_prep_pos[1][1]+self.num_pick[idx]*0.01<self.target_prep_pos[1][1] or self.hand_facing=='sweep':
				self.hand_facing=hand_facing
				self.hand_mode=hand_mode
				self.target_prep_pos=target_prep_pos
				self.target_pos=target_pos
				self.target_id=idx
		self.num_pick[self.target_id]+=1
	def close_hand(self):
		if self.hand_mode=='power':
			self.hand.setCommand(power_close_hand)
		elif self.hand_mode=='precision':
			self.hand.setCommand(precision_close_hand)
		else:
			print 'wrong hand mode!!!'
	def open_hand(self):
		if self.hand_mode=='power':
			self.hand.setCommand(power_open_hand)
		elif self.hand_mode=='precision':
			self.hand.setCommand(precision_open_hand)
		else:
			print 'wrong hand mode!!!'


	def check_target(self):
		remove_list=[]
		for i in range(len(self.waiting_list)):
			p=self.sim.world.rigidObject(self.waiting_list[i]).getTransform()[1]
			if p[2]<0.45 or self.num_pick[self.waiting_list[i]]>3:
				remove_list.append(self.waiting_list[i])
			if p[0]<0.25 and p[0]>-0.25 and p[1]<0.25 and p[1]>-0.25:
				print 'get a point!'
				self.score=self.score+1
		for i in remove_list:
			self.waiting_list.remove(i)
	def go_to(self,controller,current_pos,goal_pos,speed=move_speed):
			t=vectorops.distance(current_pos[1],goal_pos[1])/speed
			send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);		
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

