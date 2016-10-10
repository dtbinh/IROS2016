from klampt import *
from klampt.math import vectorops,so3,se3,so2
from moving_base_control import *
from reflex_control import *
# import time
import math
c_hand=0.3
o_hand=0.53
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
ready_pos=(face_down,[0.0,0,0.25])
drop_pos=(face_down,[0.5,0,0.6])


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
		self.current_target_pos=[0,0,0]
		self.waiting_list=range(self.num_ball)
		self.score=0
		self.print_flag=0
		self.tooclose=False;


		#get references to the robot's sensors (not properly functioning in 0.6.x)
		self.f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.contact_sensors = self.f1_proximal_takktile_sensors + self.f1_distal_takktile_sensors + self.f2_proximal_takktile_sensors + self.f2_distal_takktile_sensors + self.f3_proximal_takktile_sensors + self.f3_distal_takktile_sensors
		self.verbose = False
	def __call__(self,controller):
		sim = self.sim
		sim.updateWorld()
		# xform = self.base_xform
		#turn this to false to turn off print statements
		ReflexController.verbose = False
		ReflexController.__call__(self,controller)
		time=sim.getTime();
		current_pos=get_moving_base_xform(self.robot)
		#finger contact
		if self.verbose: 
			try:
				f1_contact,f2_contact,f3_contact = self.contact_measurements()
				print "Contact sensors"
				print "  finger 1:",[int(v) for v in f1_contact]
				print "  finger 2:",[int(v) for v in f2_contact]
				print "  finger 3:",[int(v) for v in f3_contact]
			except:
				pass
		
		#controller state machine
		if self.print_flag==1:
			print "State:",self.state
			self.print_flag=0
		if self.state == 'idle':
			if not self.at_destination(current_pos,start_pos):
				self.go_to(controller,current_pos,start_pos)
				self.open_hand()
			else:
				print '-----------------------------------------'
				self.check_target()
				if len(self.waiting_list)>0:
					print "Score:",self.score,"/",self.num_ball
					self.set_state('find_target')
				elif len(self.waiting_list)<1:
					print "Finished! Total score is",self.score,"/",self.num_ball
	                                print time
	                                exit()
		elif self.state=='find_target':
			self.target=self.find_target();
			self.go_to_fast(controller,current_pos,(self.target[0],start_pos[1]))
			self.set_state('pick_target')
		elif self.state == 'pick_target':	
			if time<self.last_state_end_t+1:
				if self.tooclose:
					self.go_to(controller,current_pos,(self.target[0],vectorops.add(start_pos[1],[0,0,-.35])))
				else:
					self.go_to(controller,current_pos,(self.target[0],vectorops.add(self.target[1],[0,0,0.2])))
				if self.target_is_not_moving():
					pass
				else:
					self.set_state('find_target')
					print 'target is moving!!!'
			elif time<self.last_state_end_t+1.5:
				self.go_to(controller,current_pos,self.target)
			elif time<self.last_state_end_t+2:
				#this is needed to stop at the current position in case there's some residual velocity
				controller.setPIDCommand(controller.getCommandedConfig(),[0.0]*len(controller.getCommandedConfig()))
				self.close_hand()
			else:
				self.set_state('raising')
		elif self.state=='raising':
			raise_pos=(current_pos[0],[current_pos[1][0],current_pos[1][1],0.7])
			if not self.at_destination(current_pos,raise_pos):
				self.go_to(controller,current_pos,raise_pos)
			else:
				self.set_state('move_to_drop_position')
		elif self.state == 'move_to_drop_position':
			if not self.ball_in_hand(current_pos):
				if current_pos[1][0]<0.25:
					print current_pos[1][0]
					self.set_state('idle')
			if self.at_destination(current_pos,drop_pos):
				self.set_state('drop')
			else:
				self.go_to(controller,current_pos,drop_pos)
		elif self.state=='drop':
			if time<self.last_state_end_t+0.4 :
				self.open_hand(release_hand)
			else:
				self.set_state('idle')
				
	def at_destination(self,current_pos,goal_pos):
		if vectorops.distance(current_pos[1],goal_pos[1])<0.05:
			return True
		else:	
			return False
	def find_target(self):
		self.current_target=self.waiting_list[0]
		best_p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		self.current_target_pos=best_p
		for i in self.waiting_list:
			p=self.sim.world.rigidObject(i).getTransform()[1]
			# print i,p[2],vectorops.distance([0,0],[p[0],p[1]])
			if p[2]>best_p[2]+0.05:
				self.current_target=i
				self.current_target_pos=p
				# print 'higher is easier!'
				best_p=p
			elif p[2]>best_p[2]-0.04 and vectorops.distance([0,0],[p[0],p[1]])<vectorops.distance([0,0],[best_p[0],best_p[1]]):
				self.current_target=i
				self.current_target_pos=p		
				best_p=p
		d_x=-0.02
		face_down=face_down_forward
		self.tooclose = False;
		if best_p[0]>0.15:
			d_x=-0.02
			face_down=face_down_backward
			self.tooclose = True;
			print 'too close to the wall!!'
		elif best_p[0]<-0.15:
			d_x=0.02
			self.tooclose = True;
			print 'too close to the wall!!'
		#here is hardcoding the best relative position for grasp the ball

		target=(face_down,vectorops.add(best_p,[d_x,0,0.14]))
		if self.tooclose:
			balllocation = best_p
			handballdiff = vectorops.sub(ready_pos[1],best_p)
			axis = vectorops.unit(vectorops.cross(handballdiff,ready_pos[1]))
			angleforaxis = -1*math.acos(vectorops.dot(ready_pos[1],handballdiff)/vectorops.norm(ready_pos[1])/vectorops.norm(handballdiff))+math.pi
			angleforaxis = so2.normalize(angleforaxis)
			#if angleforaxis>math.pi:
			#	angleforaxis=angleforaxis-2*math.pi
			adjR = so3.rotation(axis, angleforaxis)
			print balllocation
			print vectorops.norm(ready_pos[1]),vectorops.norm(handballdiff),angleforaxis
			target=(adjR,vectorops.add(best_p,vectorops.div(vectorops.unit(handballdiff),7)))
			

		# print self.current_target	
		# print 'find target at:',self.current_target_pos	
		return target
	def go_to(self,controller,current_pos,goal_pos):
		t=vectorops.distance(current_pos[1],goal_pos[1])/move_speed
		send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],t);
	def go_to_fast(self,controller,current_pos,goal_pos):
		send_moving_base_xform_linear(controller,goal_pos[0],goal_pos[1],.1);
		print "GO TO FAST"
	def close_hand(self):
		self.hand.setCommand(close_hand)
	def open_hand(self,hand_config=open_hand):
		self.hand.setCommand(hand_config)
	def check_target(self):
		p=self.sim.world.rigidObject(self.current_target).getTransform()[1]
		if p[0]>0.25 or p[0]<-0.25 or p[1]>0.25 or p[1]<-0.25:
			self.waiting_list.remove(self.current_target)
		if p[0]<0.95 and p[0]>0.45 and p[1]<0.25 and p[1]>-0.25:
			self.score=self.score+1
		#print self.waiting_list
		print len(self.waiting_list)," balls remaining"
	def set_state(self,state_name):
		self.state=state_name
		self.last_state_end_t=self.sim.getTime()
		self.print_flag=1
	def ball_in_hand(self,current_pos):
		if vectorops.distance(current_pos[1],self.sim.world.rigidObject(self.current_target).getTransform()[1])<0.2:
			return True
		else:
			return False
	def target_is_not_moving(self):

		if vectorops.distance(self.current_target_pos,self.sim.world.rigidObject(self.current_target).getTransform()[1])<0.04:
			return True
		else:
			return False


def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""
	return StateMachineController(sim,hand,dt)
