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
from dobot_msgs.srv import *
class PTPCMDServiceHandler:
        last_command = None
        def __init__(self):
                self.last_command = None
        def PTPCmdServer(self):
                # initialize node
                rospy.init_node('SetPTPServer')
                ptp_server = rospy.Service('SetPTPCmds', SetPTPCmd, self.handlePTPCmds)
                # spin() simply keeps python from exiting until this node is stopped
                rospy.spin()

                
        def handlePTPCmds(self, data):
                if self.last_command != None and self.last_command == data :
                        self.last_command = None
                        response = SetPTPCmdResponse()
                else:
                        self.last_command = data
                        msg = data
                #	msg = SetPTPCmd()
                #        msg.ptpMode = 0
                #        msg.x = 0.0
                #        msg.y = 200.0
                #        msg.z = 0.0
                #        msg.r = 0.0
                #        msg.isQueued = False
                        rospy.loginfo(msg)
                        rospy.wait_for_service('/Dobot_Loader/SetPTPCmd')
                        ptp_service = rospy.ServiceProxy('/Dobot_Loader/SetPTPCmd', SetPTPCmd)
                        response = ptp_service(msg.ptpMode, msg.x, msg.y, msg.z, msg.r, msg.isQueued)
                        rospy.loginfo(response)
                return response

if __name__ == '__main__':
	ptp_service_handler = PTPCMDServiceHandler()
	ptp_service_handler.PTPCmdServer()
