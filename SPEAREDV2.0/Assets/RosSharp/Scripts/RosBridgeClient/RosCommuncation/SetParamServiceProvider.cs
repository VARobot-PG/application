
using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Rosapi;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class SetParamServiceProvider : MonoBehaviour
{
    public RosConnector RosConnector;
    protected RosSocket RosSocket;
    public string ServiceEndpoint = "/rosapi/set_param";
    public string VariableName = "/my_own/variable";
    public string VariableValue = "test.value";
    public Text textField;
    // Start is called before the first frame update
    void Start()
    {
        this.RosSocket = this.RosConnector.RosSocket;
    }

    // Update is called once per frame
    void Update()
    {
        this.VariableValue = textField.text.Trim();
    }
    public void transferParam()
    {
        transferParam(this.ServiceEndpoint, this.VariableName, this.VariableValue);
    }
    public void transferParam(string endpoint, string paramName, string paramValue)
    {
        RosSocket.CallService<SetParamRequest, SetParamResponse>(endpoint, SetParamHandler, new SetParamRequest(paramName, JsonConvert.SerializeObject(paramValue)));
    }
    private void SetParamHandler(SetParamResponse serviceResponse)
    {
        Debug.Log($"received SetParam Response: {serviceResponse.ToString()}");
    }
}
