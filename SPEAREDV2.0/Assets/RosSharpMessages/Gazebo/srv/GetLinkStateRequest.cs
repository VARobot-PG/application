/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.Gazebo
{
    public class GetLinkStateRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "gazebo_msgs/GetLinkState";

        public string link_name;
        //  name of link
        //  link names are prefixed by model name, e.g. pr2::base_link
        public string reference_frame;
        //  reference frame of returned information, must be a valid link
        //  if empty, use inertial (gazebo world) frame
        //  reference_frame names are prefixed by model name, e.g. pr2::base_link

        public GetLinkStateRequest()
        {
            this.link_name = "";
            this.reference_frame = "";
        }

        public GetLinkStateRequest(string link_name, string reference_frame)
        {
            this.link_name = link_name;
            this.reference_frame = reference_frame;
        }
    }
}