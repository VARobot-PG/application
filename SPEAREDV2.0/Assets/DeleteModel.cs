using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Gazebo;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeleteModel : MonoBehaviour
{
    public RosConnector RosConnector;
    protected RosSocket RosSocket;
    public string ServiceEndpoint = "/gazebo/delete_model";
    
    
    // Start is called before the first frame update
    void Start()
    {
        this.RosSocket = this.RosConnector.RosSocket;
    }


    // Update is called once per frame
    void Update()
    {
        
    }
    
    public void deleteModels(List<string> modelNames)
    {
        foreach (String modelName in modelNames)
        {
            Debug.Log($"Sending Request to delete Model {modelName} via {ServiceEndpoint}");
            string responseValue = RosSocket.CallService<DeleteModelRequest, DeleteModelResponse>(ServiceEndpoint, serviceResponseHandler, new DeleteModelRequest(modelName));
        }
    }

    private void serviceResponseHandler(DeleteModelResponse t)
    {
        Debug.Log($"Received DeleteModelResponse - statusMessage: {t.status_message} success: {t.success} ");
    }
}
