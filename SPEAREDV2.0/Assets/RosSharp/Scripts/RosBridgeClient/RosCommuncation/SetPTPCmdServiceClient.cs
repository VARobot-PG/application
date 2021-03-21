using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
namespace RosSharp.RosBridgeClient
{
    public class SetPTPCmdServiceClient : MonoBehaviour
    {
        public KeyCode key;
        public string serviceName = "dobot_msgs/SetPtpCmd";
        public int ptpMode = 0;
        public float x = 0f;
        public float y = 200f;
        public float z = 0f;
        public float r = 0f;
        public bool isQueued = true;
        public TextMeshProUGUI tmpro;
        public MarkerHandler markerHandler;
        public float batchSendTimeout = 0.5f;
        public RosConnector rosConnector;

        // Start is callexd before the first frame update
        void Start()
        {
            changeTextFieldContent();
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKey(key))
            {
                this.sendPTPCommand();
            }
        }
        public void sendPTPCommand()
        {
            //SetPTPCmdRequest ptpRequest = new SetPTPCmdRequest(0, 0, 200, 0, 0, false);
            byte ptpMode_byte;
            byte.TryParse("" + ptpMode, out ptpMode_byte);
            MessageTypes.Dobot.SetPTPCmdRequest request = new MessageTypes.Dobot.SetPTPCmdRequest(ptpMode_byte, x, y, z, r, isQueued);
           
            sendPTPCommand(request);
            
         }
        public void sendPTPCommand(MessageTypes.Dobot.SetPTPCmdRequest request)
        {
            RosSocket rosSocket = this.rosConnector.RosSocket;
            markerHandler.clearMarkers();
            Debug.Log($"SendPTPCommand({request.ptpMode},{request.x},{request.y},{request.z},{request.r},{request.isQueued})");
            rosSocket.CallService<MessageTypes.Dobot.SetPTPCmdRequest, MessageTypes.Dobot.SetPTPCmdResponse>(serviceName, responseHandler, request);
        }
       
        public void sendPTPCommands(List<MessageTypes.Dobot.SetPTPCmdRequest> requests)
        {
            foreach (MessageTypes.Dobot.SetPTPCmdRequest request in requests)
            {
                sendPTPCommand(request);
            }
        }
        public void responseHandler(MessageTypes.Dobot.SetPTPCmdResponse response)
        {
            Debug.Log("SetPTP Response: " + response.queuedCmdIndex + " result: " + response.result);
        }
        public void onChangedX(float x)
        {
            this.x = (int) x;
            changeTextFieldContent();
        }
        public void onChangedY(float y)
        {
            this.y = (int)y;
            changeTextFieldContent();
        }
        public void onChangedZ(float z)
        {
            this.z = (int)z;
            changeTextFieldContent();
        }
        public void onChangedPTPMode(float ptpMode)
        {
            this.ptpMode = (int)ptpMode;
            changeTextFieldContent();
        }
        private void changeTextFieldContent()
        {
            string text = "Mode: " + ptpMode + ", X: " + x + ", Y: " + y + ", Z: " + z;
            tmpro.text = text;
        }
    }

}