#!/usr/bin/env python

# Siemens AG, 2018
# Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from detection_msgs.srv import *
from detection_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
class AddDetectedObjectServiceHandler:
        last_command = None
        counter = 0
        def __init__(self):
                self.last_command = None
                self.counter = 1
        def AddDetectedObjectServer(self):
                # initialize node
                rospy.init_node('AddDetectedObjectServer')
                ptp_server = rospy.Service('AddDetectedObjects', AddDetectedObject, self.handleAddDetectedObjects)
                # spin() simply keeps python from exiting until this node is stopped
                rospy.spin()

                
        def handleAddDetectedObjects(self, data):
                msg = data
                self.counter = self.counter + 1
        #	msg = SetPTPCmd()
        #        msg.ptpMode = 0
        #        msg.x = 0.0
        #        msg.y = 200.0
        #        msg.z = 0.0
        #        msg.r = 0.0
        #        msg.isQueued = False
                pose = Pose()
                rospy.loginfo(msg)
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                req = SpawnModelRequest()
                f = open('/root/catkin_ws/src/gazebo_simulation_scene/sdf/box.sdf','r')
                sdff = f.read()
                model_xml = sdff
                req.model_name = "ADO" + str(self.counter)
                req.model_xml = model_xml
                pose.orientation.w = 1
                pose.position = data.obj.startPoint
                if pose.position.x == 0.0 and pose.position.y == 0.0 and pose.position.z == 0.0:
                    pose.position.x = 132.073
                    pose.position.y = 732.953
                    pose.position.z = 63.937
                pose.position.x = pose.position.x / 1000.0
                pose.position.y = pose.position.y / 1000.0
                pose.position.z = pose.position.z / 1000.0
                # should this be unique so ros_control can use each individually?
                req.robot_namespace = "/Dobot_Loader/"
                req.initial_pose = pose
                response_spawn = SpawnModelResponse()
                response_spawn = spawn_model(req)
                response_ao = AddDetectedObjectResponse()
                response_ao.success = True
                rospy.loginfo("response from SpawnSDF:" + str(response_spawn))
                return response_ao

if __name__ == '__main__':
	ado_service_handler = AddDetectedObjectServiceHandler()
	ado_service_handler.AddDetectedObjectServer()
