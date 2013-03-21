#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_environment_perception_intern
# \note
#   ROS package name: cob_3d_mapping_demonstrator
#
# \author
#   Author: Joshua Hampp
#
# \date Date of creation: 03/2012
#
# \brief
#   Implementation of ROS node for script_server.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time

import roslib
roslib.load_manifest('cob_camera_alignment')
import rospy
import yaml, time
import smach
import smach_ros
import shutil
import numpy as np

from start_calib import execute_button_commands
from BasicIO import *

from sensor_msgs.msg import Imu, JointState, ChannelFloat32

calib_table= []
calib_beta = 0

sss = simple_script_server()

class StateListener:
	use = False
	
	def __init__(self, name):
		rospy.Subscriber(name, ChannelFloat32, self.callback)

	def callback(self, data):
		if self.use:
			self.vec += data.values[0]
			self.num+= 1
			if self.num>20:
				self.use = False

	def getval(self):
		self.vec = 0
		self.num = 0
		self.use = True
		while self.use and not rospy.is_shutdown():
			time.sleep(0.1)
		return self.vec/self.num

class JointStateListener:
	use = False
	
	def __init__(self, name):
		rospy.Subscriber(name, JointState, self.callback)

	def callback(self, data):
		if self.use:
			i=-1
			for n in data.name:
				i+=1
				if n!='tray_joint': continue
				self.vec += data.position[i]
				self.num+= 1
			if self.num>20:
				self.use = False

	def getval(self):
		self.vec = 0
		self.num = 0
		self.use = True
		while self.use and not rospy.is_shutdown():
			time.sleep(0.1)
		return self.vec/self.num

joint_state = StateListener('/state')
joint_state2 = JointStateListener('/joint_states')

class CalibrationNull(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['failed', 'succeeded'])

	def execute(self, userdata):
		global calib_table
		global calib_beta
		global joint_state
		global joint_state2

		try:
			ebc = execute_button_commands()
			ebc.start = joint_state2.getval()
			print "start: ", ebc.start
			ebc.execute_nullposition(None)
		except Exception as e:
			print e
			return 'failed'

		if ebc.valid:
			calib_table= [ [0, joint_state.getval()] ]	#null position
			calib_beta = ebc.deltas[0]			#beta offset
			return 'succeeded'
		return 'failed'

class SaveBeta(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['failed', 'succeeded'])

	def execute(self, userdata):
		global calib_beta
		if not rospy.has_param('~calib_file'):
			return 'failed'
			
		fn = rospy.get_param('~calib_file')
		shutil.copyfile(fn,fn+".backup")

		f = open(fn,'r')
		lines=[]
		for l in f.readlines(): lines.append(l)
		f.close()

		found = False
		f = open(fn,'w')
		for l in lines:
			if l.find("mobina_calib_beta")>=0:
				f.write('  <property name="mobina_calib_beta" value="'+str(calib_beta)+'" />\n')
				found = True
			else: f.write(l)
		f.close()
		
		if not found: return 'failed'
		return 'succeeded'

class LinearInterpolationValues(smach.State):
	use = False
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['failed', 'succeeded'])
		self.actor = rospy.get_param('~actor')

	def callback(self,data):
		if self.use:
			self.vec += np.array([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])
			self.num += 1
			if self.num>20:
				self.use = False

	def getval(self):
		self.vec = np.array([0., 0., 0.])
		self.num = 0
		self.use = True
		while self.use and not rospy.is_shutdown():
			time.sleep(0.1)
		return self.vec/np.linalg.norm(self.vec)

	def moveto(self,j):
		r = sss.move(self.actor,[[j]])
		if r.get_state()!=3: return False
		sss.sleep(0.5)
		return True

	def execute(self, userdata):
		global calib_table
		global joint_state

		if not rospy.has_param('~calib_file_controller'):
			return 'failed'

		rospy.Subscriber("/imu/data_raw", Imu, self.callback)

		v = self.getval()

		delta = 0.15
		start = joint_state2.getval()

		for f in [1,-1]:
			new  = start
			while True:
				new += f*delta
				if not self.moveto(new): break
				w = self.getval()
				alpha = math.acos(np.dot(w, v))
				print w, alpha
				calib_table.append( [f*alpha, joint_state.getval()] )

		f = open(rospy.get_param('~calib_file_controller'),'w')
		l = sorted(calib_table, key=lambda c: c[0])
		print l
		f.write('calibration_pos: '+str(l)+'\n')
		f.close()
		
		return 'succeeded'

class Calibration(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['succeeded','failed'])
		with self:		
			smach.StateMachine.add('START', Light('blue'), transitions={'succeeded':'CALIB_NULLPOS'})

			smach.StateMachine.add('CALIB_NULLPOS', CalibrationNull(),
				transitions={'succeeded':'SAVE_BETA','failed':'FAILURE'})

			smach.StateMachine.add('SAVE_BETA', SaveBeta(),
				transitions={'succeeded':'INTERPOLATION','failed':'FAILURE'})

			smach.StateMachine.add('INTERPOLATION', LinearInterpolationValues(),
				transitions={'succeeded':'SUCCESS','failed':'FAILURE'})

			smach.StateMachine.add('SUCCESS', Light('green'), transitions={'succeeded':'failed'})

			smach.StateMachine.add('FAILURE', Light('red'), transitions={'succeeded':'failed'})

if __name__ == '__main__':
	import sys
	#-----	EXECUTE SMACH				-------------------------------------------------------#
	rospy.init_node('calibration')
	
	sm = Calibration()

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('CALIBRATION', sm, 'CALIBRATION')
	smach_viewer.start()

	#smach_thread = threading.Thread(sm.execute())
	#smach_thread.start()
	sm.execute()

	#rospy.spin()
	smach_viewer.stop()

