/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Gazebo
{
    public class ContactState : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "gazebo_msgs/ContactState";

        public string info;
        //  text info on this contact
        public string collision1_name;
        //  name of contact collision1
        public string collision2_name;
        //  name of contact collision2
        public Wrench[] wrenches;
        //  list of forces/torques
        public Wrench total_wrench;
        //  sum of forces/torques in every DOF
        public Vector3[] contact_positions;
        //  list of contact position
        public Vector3[] contact_normals;
        //  list of contact normals
        public double[] depths;
        //  list of penetration depths

        public ContactState()
        {
            this.info = "";
            this.collision1_name = "";
            this.collision2_name = "";
            this.wrenches = new Wrench[0];
            this.total_wrench = new Wrench();
            this.contact_positions = new Vector3[0];
            this.contact_normals = new Vector3[0];
            this.depths = new double[0];
        }

        public ContactState(string info, string collision1_name, string collision2_name, Wrench[] wrenches, Wrench total_wrench, Vector3[] contact_positions, Vector3[] contact_normals, double[] depths)
        {
            this.info = info;
            this.collision1_name = collision1_name;
            this.collision2_name = collision2_name;
            this.wrenches = wrenches;
            this.total_wrench = total_wrench;
            this.contact_positions = contact_positions;
            this.contact_normals = contact_normals;
            this.depths = depths;
        }
    }
}
