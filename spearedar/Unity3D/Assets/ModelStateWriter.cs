using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ModelStateWriter : MonoBehaviour
{
    public UnityEngine.Transform targetTransform;
    public UnityEngine.Transform rosTransform;
    public UnityEngine.Vector3 newTargetTransformPos;
    public UnityEngine.Vector3 newROSTransformPos;
    public bool updateTransform = false;
    public CoordinateTargetSelector coordinateTargetSelector;
    // Start is called before the first frame update
    void Start()
    {
        newTargetTransformPos = UnityEngine.Vector3.zero;
        newROSTransformPos = UnityEngine.Vector3.zero;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (updateTransform)
        {
            //if (newTargetTransformPos != UnityEngine.Vector3.zero && newTargetTransformPos != targetTransform.localPosition)
            //{
                rosTransform.localPosition = newROSTransformPos;

                //Debug.Log($"Updateing {targetTransform.name} from {targetTransform.localPosition} to {newTargetTransformPos}");
                targetTransform.localPosition = newTargetTransformPos;
           // }
        }
    }
    public void Write(RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose pose, RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist twist)
    {
        UnityEngine.Vector3 newPosition;
       
            newROSTransformPos = new UnityEngine.Vector3((float)pose.position.x, (float)pose.position.y, (float)pose.position.z); 
            newROSTransformPos = newROSTransformPos * 1000f;

            newPosition = coordinateTargetSelector.transformVectorRos2UnityLocal(newROSTransformPos);

        newTargetTransformPos = newPosition;
            
    }
}
