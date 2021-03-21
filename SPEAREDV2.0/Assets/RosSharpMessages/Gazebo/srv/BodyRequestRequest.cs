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
    public class BodyRequestRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "gazebo_msgs/BodyRequest";

        public string body_name;
        //  name of the body requested. body names are prefixed by model name, e.g. pr2::base_link

        public BodyRequestRequest()
        {
            this.body_name = "";
        }

        public BodyRequestRequest(string body_name)
        {
            this.body_name = body_name;
        }
    }
}
