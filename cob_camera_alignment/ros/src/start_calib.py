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
import actionlib
import yaml

from cob_camera_alignment.msg import *
from cob_srvs.srv import *
from simple_script_server import *

sss = simple_script_server()

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class execute_button_commands():
  ## Initializes the actionlib interface of the script server.
  #
  def __init__(self):
    self.ns_global_prefix = "/cob_calibration"

    self.no_of_fr = rospy.get_param('~number_of_frames', 20)
    self.var = rospy.get_param('~variance')
    self.max_delta = rospy.get_param('~max_delta_rad')

    self.actor = rospy.get_param('~actor')
    self.default_joints = rospy.get_param('~default_joints')
    self.joint_index = rospy.get_param('~joint_index')

    self.step_size = rospy.get_param('~step_size')
    self.range_min = rospy.get_param('~range_min')
    self.range_max = rospy.get_param('~range_max')

    self.script_action_server = actionlib.SimpleActionServer("cob_calibration", ScriptAction, self.execute_cb, False)
    self.script_action_server.register_preempt_callback(self.execute_stop)
    self.script_action_server.start()
    self.trigger_client = actionlib.SimpleActionClient('/start_measurements', StartMeasurementsAction)

    rospy.Service('/cob_calibration_controller/nullposition', Trigger, self.execute_nullposition)
    rospy.Service('/cob_calibration_controller/movesteps', Trigger, self.execute_movesteps)

#------------------- Actionlib section -------------------#
  ## Executes actionlib callbacks.
  #
  # \param server_goal ScriptActionGoal
  #
  def execute_cb(self, server_goal):
    server_result = ScriptActionResult().result
    if server_goal.function_name == "nullposition":
      ret=self.execute_nullposition()
    elif server_goal.function_name == "stop":
      ret=self.execute_stop()
    else:
      rospy.logerr("function <<%s>> not supported", server_goal.function_name)
      self.script_action_server.set_aborted(server_result)
      return

    #server_result.error_code = handle01.get_error_code()
    #if server_result.error_code == 0:
    #  rospy.logdebug("action result success")
    if self.script_action_server.is_active():
      self.script_action_server.set_succeeded(server_result)
    #else:
    #  self.script_action_server.set_aborted(server_result)
    #else:
    #  rospy.logerr("action result error")
    #  self.script_action_server.set_aborted(server_result)

  def move_to(self, pos):
    j = self.default_joints
    j[self.joint_index] = pos
    sss.move(self.actor,[j])
    rospy.sleep(0.5)

  def measure(self):
    goal = StartMeasurementsGoal()
    goal.number_of_frames = self.no_of_fr
    self.trigger_client.send_goal(goal)
    if not self.trigger_client.wait_for_result(rospy.Duration.from_sec(0.2*goal.number_of_frames)):
      print "measurement failed"
      return 100
    res = self.trigger_client.get_result()
    if len(res.deltas)<1:
      print "measurement invalid"
      return 100
    return res.deltas[0]

  def measure_at(self, pos):
    self.move_to(pos)
    return self.measure()

  def iterate1(self, mid, r):
    if r<self.var: return [mid+r/2, self.measure_at(mid+r/2)]

    d = [self.measure_at(mid-r), self.measure_at(mid+r)]
    print d
    print [mid-r, mid, mid+r]
    if abs(d[0])<abs(d[1]):
       return self.iterate1(mid-r/2,r/2+self.var*0.3)
    else:
       return self.iterate1(mid+r/2,r/2+self.var*0.3)



  def execute_nullposition(self, req):
    print "calibrating null-position..."
    d = self.iterate1(0,self.max_delta)
    print "null-position at ",d[0]
    print "literal difference ",d[1]," rad"
    return TriggerResponse()

  def execute_stop(self):
    print "stop"
    self.script_action_server.set_preempted()
    sss.stop("cob_3d_mapping_demonstrator")



  def execute_movesteps(self, req):
    print "calibrating null-position..."
    m = {}
    for s in [x * self.step_size for x in range(self.range_min, self.range_max)]:
    	d=self.measure_at(s)
    	m[s]=d
    	print "deviation at ",s," is ",d," rad"
    print m
    return TriggerResponse()

  def execute_stop(self):
    print "stop"
    self.script_action_server.set_preempted()
    sss.stop("cob_3d_mapping_demonstrator")

## Main routine for running the script server
#
if __name__ == '__main__':
  rospy.init_node('start_calib')
  execute_button_commands()
  rospy.loginfo("calibration is ready")
  rospy.spin()
