using UnityEngine;
using System.Collections;
//using RosSharp.RosBridgeClient.MessageTypes.Gazebo;
namespace RosSharp.RosBridgeClient
{
    public class PausePhysics : MonoBehaviour
    {
        public RosConnector RosConnector;
        protected RosSocket RosSocket;
        public string ServiceEndpoint = "/gazebo/set_physics_properties";
        public string PauseEndpoint = "/gazebo/pause_physics";
        public string UnpauseEndpoint = "/gazebo/unpause_physics";
        protected MessageTypes.Gazebo.ODEPhysics DefaultODEPhysics =
            new MessageTypes.Gazebo.ODEPhysics();
        protected MessageTypes.Gazebo.SetPhysicsPropertiesRequest DefaultPhysics;
        public bool isPhysicsPaused = false;

        public void SetRosConnector(RosConnector newCon)
        {
            if (newCon != null)
            {
                this.RosConnector = newCon;
                this.RosSocket = this.RosConnector.RosSocket;
            }
        }

        // Start is called before the first frame update
        void Start()
        {
            
            SetRosConnector(this.RosConnector);
            
           // StartCoroutine("testCoroutine");

            //this triggers a call to get the current settings every 5 seconds
            InvokeRepeating("GetCurrentParams", 1, 5);
            this.GetCurrentParams();
        }

        public void responseHandler(MessageTypes.Gazebo.SetPhysicsPropertiesResponse response)
        {
            if (response.success)
            {
                Debug.Log($"PhysicsProperties update succeeded. Message text " +
                    $"was: '{response.status_message}'");
            }
            else
            {
                Debug.Log($"PhysicsProperties update FAILED. Message text " +
                    $"was: '{response.status_message}'");
            }
        }
        
        public IEnumerator testCoroutine()
        {
            this.normalPhysics();
            yield return new WaitForSecondsRealtime(45);
            this.SimulateSlower(0.5);
            yield return new WaitForSecondsRealtime(15);
            this.normalPhysics();
            yield return new WaitForSecondsRealtime(10);
            this.SimulateFaster(2.25);
            yield return new WaitForSecondsRealtime(45);
            this.pausePhysics();
            yield return new WaitForSecondsRealtime(5);
            this.continuePhysics();
        }

        public void physiscsCallback(MessageTypes.Gazebo.GetPhysicsPropertiesResponse rep)
        {
            //create default SetPhysicsMessage
            MessageTypes.Gazebo.SetPhysicsPropertiesRequest newDefault =
                new MessageTypes.Gazebo.SetPhysicsPropertiesRequest(
                    time_step: rep.time_step, max_update_rate:
                    rep.max_update_rate, gravity: rep.gravity, rep.ode_config);

            this.DefaultPhysics = newDefault;
            isPhysicsPaused = rep.pause;
        }


        public void GetCurrentParams()
        {

                RosSocket.CallService<MessageTypes.Gazebo.
                    GetPhysicsPropertiesRequest, MessageTypes.Gazebo.
                    GetPhysicsPropertiesResponse>("/gazebo/get_physics_properties", physiscsCallback,
                    new MessageTypes.Gazebo.GetPhysicsPropertiesRequest());
                // end here as the response and the actual update will be ahandled by the callback

        }

        //clones the settings provided in new messages that can be modified without changing the currently saved default
        private MessageTypes.Gazebo.SetPhysicsPropertiesRequest createCloneForPhysics(MessageTypes.Gazebo.SetPhysicsPropertiesRequest orig)
        {


            MessageTypes.Gazebo.ODEPhysics clonedOEDPhzsics = new MessageTypes.Gazebo.ODEPhysics(orig.ode_config.auto_disable_bodies, 
            orig.ode_config.sor_pgs_precon_iters,
            orig.ode_config.sor_pgs_iters,
            orig.ode_config.sor_pgs_w, 
            orig.ode_config.sor_pgs_rms_error_tol, 
            orig.ode_config.contact_surface_layer,
            orig.ode_config.contact_max_correcting_vel, 
            orig.ode_config.cfm, 
            orig.ode_config.erp, 
            orig.ode_config.max_contacts );

            MessageTypes.Gazebo.SetPhysicsPropertiesRequest clone =
                new MessageTypes.Gazebo.SetPhysicsPropertiesRequest(orig.time_step,
                orig.max_update_rate, orig.gravity, clonedOEDPhzsics);

            return clone;
        }




        //sets the current speed of the simulation, this basically allows slowing down and speeding
        // it up, this method should not be used to pause the simulation completly
        public void setSimulationSpeed(double newSpeed=0.001)
        {
            
            //create a message based on the cached params by cloning it's values


            MessageTypes.Gazebo.SetPhysicsPropertiesRequest request =
                this.createCloneForPhysics(this.DefaultPhysics);// None;
            // setting eerything else back to defaults 

            request.time_step = newSpeed;
            RosSocket.CallService<MessageTypes.Gazebo.SetPhysicsPropertiesRequest,
                MessageTypes.Gazebo.SetPhysicsPropertiesResponse>
                (ServiceEndpoint, responseHandler, request);
            

        }

        public void SimulateSlower()
        {
            this.SimulateSlower(0.5);
        }


        public void SimulateSlower(double factor = 0.5)
        {
            if(factor>1 || factor < 0)
            {
                throw new System.Exception("Illegal Arguments supplied. Only" +
                    " doubles bigger than 0 and smaller or equal to 1 are" +
                    " alllowed. ");
            }
            //get current speed 
            double defaultSpeed = 0.01;
            if (this.DefaultPhysics != null)
            {
                defaultSpeed=this.DefaultPhysics.time_step;
            }
            double newSpeed = defaultSpeed * factor;
            this.setSimulationSpeed(newSpeed);
        }

        public void SimulateFaster()
        {
            this.SimulateFaster(1.5);
        }


        public void SimulateFaster(double factor=1.5)
        {
            //get current simulation speed
            
            //multiply with factor
            //use setSimulationSpeed to set it to result
            double defaultSpeed = this.DefaultPhysics.time_step;
            double newSpeed = defaultSpeed * factor;
            this.setSimulationSpeed(newSpeed);

        }

       
        //sets the simulations settings back to the default 
        public void normalPhysics()
        {
            //note also toggles the time
            RosSocket.CallService<MessageTypes.Std.EmptyRequest,
                MessageTypes.Std.EmptyResponse>
                (UnpauseEndpoint, PauseContinueResponseHandler,
                new MessageTypes.Std.EmptyRequest());


            //set the default physics
            MessageTypes.Gazebo.SetPhysicsPropertiesRequest pauseRequest =
                new MessageTypes.Gazebo.SetPhysicsPropertiesRequest();
            RosSocket.CallService<MessageTypes.Gazebo.SetPhysicsPropertiesRequest,
                MessageTypes.Gazebo.SetPhysicsPropertiesResponse>
                (ServiceEndpoint, responseHandler, pauseRequest);
        }

        public void PauseContinueResponseHandler(MessageTypes.Std.EmptyResponse rep) {
            //literally do nothing
        }

        public void pausePhysics(){
            //
            RosSocket.CallService<MessageTypes.Std.EmptyRequest,
                MessageTypes.Std.EmptyResponse>
                (PauseEndpoint, PauseContinueResponseHandler,
                new MessageTypes.Std.EmptyRequest());

        }
        //this method is basically an alias for unpause
        public void continuePhysics()
        {
            this.unpausePhysics();
        }

        public void unpausePhysics()
        {
            RosSocket.CallService<MessageTypes.Std.EmptyRequest,
                MessageTypes.Std.EmptyResponse>
                (UnpauseEndpoint, PauseContinueResponseHandler,
                new MessageTypes.Std.EmptyRequest());

        }
    }
}
