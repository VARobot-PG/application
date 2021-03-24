using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Rosapi;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class GetParam : MonoBehaviour
{
    public RosConnector RosConnector;
    protected RosSocket RosSocket;
    public string ServiceEndpoint = "/rosapi/set_param";
    public string VariableName = "/my_own/variable";
    public Text textField;
    public string deserializedResponse;
    public bool newResponseAvailable = false;
    // Start is called before the first frame update
    void Start()
    {
        this.RosSocket = this.RosConnector.RosSocket;
    }


    // Update is called once per frame
    void Update(){
        if (newResponseAvailable)
        {
            updateUI();
            newResponseAvailable = false;
        }
    }
    public void getParam()
    {
        getParam(this.ServiceEndpoint, this.VariableName);
    }
    public void getParam(string endpoint, string paramName)
    {
        string responseValue = RosSocket.CallService<GetParamRequest, GetParamResponse>(endpoint, GetParamHandler, new GetParamRequest(paramName, JsonConvert.SerializeObject("")));
    }
    public void updateUI()
    {
        Debug.Log($"Updating UI GetParam Response: {deserializedResponse}");
        this.textField.text = deserializedResponse;
    }
    private void GetParamHandler(GetParamResponse serviceResponse)
    {
        
        Debug.Log($"received GetParam Response: {serviceResponse.ToString()}");
        if (!serviceResponse.value.Equals("")) {
            deserializedResponse = (string)JsonConvert.DeserializeObject(serviceResponse.value.Replace(",",".")); 
            Debug.Log($"deserialized Response: {deserializedResponse}");
            newResponseAvailable = true;
        }
        
    }
}
