using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ModelStateSubscriber : UnitySubscriber<RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelStates>
{
    public string[] modelNamesFromGazebo;
        public List<string> ModelNames;
        public List<ModelStateWriter> ModelStateWriters;
    public List<GameObject> generatedObjects = new List<GameObject>();
    public GameObject generatedCubePrefab;
    public UnityEngine.Transform genCubeParentTransform;
    public List<RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelState> modelsToGen = new List<RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelState>();
    public CoordinateTargetSelector coordinateTargetSelector;
    public DeleteModel deleteModel;
    public ExecuteGeneratedProgram currentExecutor;
    public GameObject LoaderSucker;
    public SuctionCupStatusSubscriber suctionCupStatusSubscriber;

        // Example ROS Cube 132.073, 732.953, 63.937
    
    // Update is called once per frame
    void Update()
    {
        if(modelsToGen.Count > 0)
        {
            foreach(RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelState modelState in modelsToGen)
            {
                if (ModelNames.Contains(modelState.model_name))
                {

                }
                else
                {
                    ModelNames.Add(modelState.model_name);
                    ModelStateWriters.Add(generateNewObject(modelState));
                }
            }
            modelsToGen.Clear();
        }
    }

        protected override void ReceiveMessage(RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelStates message)
        {
            int index;
            modelNamesFromGazebo = message.name;
            for (int i = 0; i < message.name.Length; i++)
            {
                string name = message.name[i];
                RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose pose = message.pose[i];
            // Gazebo is in meters, we think in millimeters
            /*  
            pose.position.x = pose.position.x * 1000.0f;
                pose.position.y = pose.position.y * 1000.0f;
                pose.position.z = pose.position.z * 1000.0f;
                pose.position.x = pose.position.x - differenceGazeboUnity.x;
                pose.position.y = pose.position.y - differenceGazeboUnity.y;
                pose.position.z = pose.position.z - differenceGazeboUnity.z;
            */
            Twist twist = message.twist[i];
                if (name.Contains("ADO"))
                {
                    index = ModelNames.IndexOf(name);
                //Debug.Log($"{name} x: {pose.position.x} y: {pose.position.x} z: {pose.position.z} ");
                    if (index != -1)
                    {
                        if (ModelStateWriters.Count > index)
                        {
                            ModelStateWriter msw = ModelStateWriters[index];
                            if (msw != null)
                            {
                                msw.Write(pose, twist);
                            }
                        }
                    }
                    else
                    {
                        RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelState modelstate = new RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelState(name, pose, twist, "");
                        if (modelsToGen.Exists(model => { return model.model_name.Equals(modelstate.model_name); })) { 
                        }
                        else
                        {
                        if (ModelNames.Contains(modelstate.model_name))
                        {

                        }
                        else
                        {
                            modelsToGen.Add(modelstate);
                        }
                        }
                    }
                }
            }
     }

    private ModelStateWriter generateNewObject(RosSharp.RosBridgeClient.MessageTypes.Gazebo.ModelState modelstate)
    {
        RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose pose = modelstate.pose;
        Twist twist = modelstate.twist;
        string name = modelstate.model_name;
        GameObject ado = Instantiate(generatedCubePrefab);

        generatedObjects.Add(ado);
        ado.name = name;
        ado.transform.SetParent(genCubeParentTransform, true);
        ModelStateWriter modelStateWriter = ado.AddComponent<ModelStateWriter>();
        CollisionTester collisionTester = ado.AddComponent<CollisionTester>();
        collisionTester.currentExecuter = this.currentExecutor;
        collisionTester.sucker = this.LoaderSucker;
        suctionCupStatusSubscriber.collisionTesters.Add(collisionTester);
        BoxCollider boxCollider = ado.GetComponent<BoxCollider>();
        boxCollider.isTrigger = true;
        foreach (UnityEngine.Transform child in ado.transform)
        {
            if (child.name.Contains("ROSCoords"))
            {
                child.localPosition = new UnityEngine.Vector3((float)pose.position.x, (float)pose.position.y, (float)pose.position.z);
                child.localPosition = child.localPosition * 1000f; 
                ado.transform.localPosition = coordinateTargetSelector.transformVectorRos2UnityLocal(child.localPosition);
                modelStateWriter.rosTransform = child;
            }
        }                
        
        modelStateWriter.targetTransform = ado.transform;
        modelStateWriter.updateTransform = true;
        modelStateWriter.coordinateTargetSelector = this.coordinateTargetSelector;
        
        coordinateTargetSelector.addOnClickHandlerAndBoundingBoxes(name);
        return modelStateWriter;
    }
    public void deleteModels() {
        deleteModel.deleteModels(this.ModelNames);
        foreach(GameObject go in generatedObjects)
        {
            Destroy(go);
        }
        this.ModelNames.Clear();
        this.ModelStateWriters.Clear();
        this.modelsToGen.Clear();
        this.generatedObjects.Clear();
        this.suctionCupStatusSubscriber.collisionTesters.Clear();
    }
}
