#!/usr/bin/env python
#
# Copyright 2013 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Desc: helper script for spawning models in gazebo
# Author: John Hsu, Dave Coleman
#

import rospy, sys, os, time
import string
import warnings

from gazebo_ros import gazebo_interface

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty

from pr2_controller_manager import pr2_controller_manager_interface

from cStringIO import StringIO
import sys


class Capturing(list):
    def __enter__(self):
        self._stdout = sys.stdout
        sys.stdout = self._stringio = StringIO()
        return self
    def __exit__(self, *args):
        self.extend(self._stringio.getvalue().splitlines())
        sys.stdout = self._stdout


def usage():
    print '''Commands:
    -[urdf|sdf|trimesh|gazebo] - specify incoming xml is urdf, sdf or trimesh format. gazebo arg is deprecated in ROS Hydro
    -[file|param|database] [<file_name>|<param_name>|<model_name>] - source of the model xml or the trimesh file
    -model <model_name> - name of the model to be spawned.
    -gazebo_namespace <gazebo ros_namespace> - optional: ROS namespace of gazebo offered ROS interfaces.  Defaults to /gazebo/ (e.g. /gazebo/model_joint_position).
    -J <joint_name joint_position> - optional: initialize the specified joint at the specified value
    '''
    sys.exit(1)

class SetModelPosition():
    def __init__(self):
        self.param_name              = ""
        self.model_name              = ""
        self.gazebo_namespace        = "/gazebo"
        self.joint_names             = rospy.get_param('~initial_joint_names',[])
        self.joint_positions         = rospy.get_param('~initial_joint_positions',[])

        self.left_arm_tuck = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
        self.right_arm_tuck = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]
        self.right_arm_ready_to_grasp = [-1.2780958803182516, 0.00979129625067987, 0.2767535259540539, -2.0621385930423557, -3.009094415608935, -0.6072794764131304, -4.783258604066159]
        self.left_arm_tuck_alone = [0.21685427439134308, 1.2614225287545082, 1.7892803096566166, -1.6871467538615672, -1.771813039429885, -0.25594707189242527, -0.045095787736748605]

        self.left_arm_names = ["l_shoulder_pan_joint"
              , "l_shoulder_lift_joint"
              , "l_upper_arm_roll_joint"
              , "l_elbow_flex_joint"
              , "l_forearm_roll_joint"
              , "l_wrist_flex_joint"
              , "l_wrist_roll_joint"]
        self.right_arm_names = ["r_shoulder_pan_joint"
              , "r_shoulder_lift_joint"
              , "r_upper_arm_roll_joint"
              , "r_elbow_flex_joint"
              , "r_forearm_roll_joint"
              , "r_wrist_flex_joint"
              , "r_wrist_roll_joint"]        

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-h' or sys.argv[i] == '--help' or sys.argv[i] == '-help':
            usage()
            sys.exit(1)
          if sys.argv[i] == '-J':
            if len(sys.argv) > i+2:
              self.joint_names.append(sys.argv[i+1])
              self.joint_positions.append(float(sys.argv[i+2]))
            else:
              print "Error: must specify a joint name and joint value pair"
              sys.exit(0)
          if sys.argv[i] == '-param':
            if len(sys.argv) > i+1:
              self.param_name = sys.argv[i+1]
          if sys.argv[i] == '-model':
            if len(sys.argv) > i+1:
              self.model_name = sys.argv[i+1]
          if sys.argv[i] == '-gazebo_namespace':
            if len(sys.argv) > i+1:
              self.gazebo_namespace = sys.argv[i+1]

        if self.model_name == "":
          print "Error: you must specify model name"
          sys.exit(0)


    def set_controllers_state(self, controllers, start_controller):
      print "Dealing with " + str(controllers)

      # Need to activate physics or the calls to pr2_controller_manager will wait till end of time
      self.set_paused_physics(False)
      
      for controller in controllers:
        if start_controller:
          pr2_controller_manager_interface.start_controller(controller)
        else:
          pr2_controller_manager_interface.stop_controller(controller)
            
      # Deactivate physics to make more changes (eventually)
      self.set_paused_physics(True)

      print "Done with controllers states"

    def set_configuration(self, controllers, joint_names, joint_positions):
        # Need to stop controllers in order to change configuration, otherwise the controllers
        # Avoid changing the configuration
        self.set_controllers_state(controllers, False)

        # set model configuration before unpause if user requested
        if len(joint_names) != 0:
          try:
            success = gazebo_interface.set_model_configuration_client(
              self.model_name, self.param_name, joint_names, joint_positions, self.gazebo_namespace)
            print "Model configuration changed"
          except rospy.ServiceException, e:
            print "set model configuration service call failed: %s"%e

        # Re-enable controllers to use the robot
        self.set_controllers_state(controllers, True)

    def set_paused_physics(self, pause):
      if pause:
        unpause = ""
      else:
        unpause = 'un'

      rospy.wait_for_service(('%s/'+unpause+'pause_physics')%(self.gazebo_namespace))
      try:
        unpause_physics = rospy.ServiceProxy('%s/unpause_physics'%(self.gazebo_namespace), Empty)
        unpause_physics()
      except rospy.ServiceException, e:
        print "unpause physics service call failed: %s"%e

    def callSpawnService(self):
        if rospy.is_shutdown():
          sys.exit(0)

        #self.set_configuration(["head_traj_controller"], self.joint_names, self.joint_positions)

        left_arm_joint_positions = self.left_arm_tuck_alone
        #self.set_configuration(["l_arm_controller"], left_arm_names, left_arm_joint_positions)

        right_arm_joint_positions = self.right_arm_ready_to_grasp
        #self.set_configuration(["r_arm_controller"], right_arm_names, right_arm_joint_positions)

        controllers = ["head_traj_controller", "r_arm_controller","l_arm_controller"]
        joint_names = self.joint_names + self.left_arm_names + self.right_arm_names
        joint_positions = self.joint_positions + left_arm_joint_positions + right_arm_joint_positions

        self.set_configuration(controllers, joint_names, joint_positions)
        self.set_paused_physics(False)

        # Do it again to avoid collisions?
        time.sleep(2) 

        self.set_configuration(controllers, joint_names, joint_positions)
        self.set_paused_physics(False)

        time.sleep(2) 

        self.set_configuration(controllers, joint_names, joint_positions)
        self.set_paused_physics(False)

        time.sleep(2) 

        self.set_configuration(controllers, joint_names, joint_positions)
        self.set_paused_physics(False)

        pr2_controller_manager_interface.list_controllers()

        return

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print usage()
    else:
        print("model_joint_position script started") # make this a print incase roscore has not been started
        rospy.init_node('model_joint_position')
        sm = SetModelPosition()
        sm.parseUserInputs()
        sm.callSpawnService()
