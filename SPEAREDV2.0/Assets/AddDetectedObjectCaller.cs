using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Detection;
using RosSharp.RosBridgeClient.MessageTypes.Rosapi;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
public class AddDetectedObjectCaller : MonoBehaviour
{
    public RosConnector RosConnector;
    protected RosSocket RosSocket;
    public string ServiceEndpoint = "/AddDetectedObjects";
    // Start is called before the first frame update
    void Start()
    {
        this.RosSocket = this.RosConnector.RosSocket;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void addDetectedObject()
    {
        AddDetectedObjectRequest adr = new AddDetectedObjectRequest();
        RosSocket.CallService<AddDetectedObjectRequest, AddDetectedObjectResponse>(this.ServiceEndpoint, responseHandler, adr);
        Debug.Log($"sent AddDetectedObjectRequest to {ServiceEndpoint}");
    }

    private void responseHandler(AddDetectedObjectResponse t)
    {
        Debug.Log($"received AddDetectedObjectResponse");
    }
}
