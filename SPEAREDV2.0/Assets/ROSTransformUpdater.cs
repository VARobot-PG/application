using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ROSTransformUpdater : MonoBehaviour
{
    public Transform rosTransform;
    public CoordinateTargetSelector transformer;
    public Vector3 oldPosition;
    public Transform ownTransform;
    public MeshRenderer meshRenderer;
    public Material outOfRangeColorPrefab;
    public bool isOutOfBounds;
    public Material selectedColorPrefab;
    public Vector3 oldRosTransformPosition;
    public Text coordText;
    public String coordPrefix = "";
    public bool useROSTransform = true;

    // Start is called before the first frame update
    void Start()
    {
        oldPosition = ownTransform.localPosition;
        isOutOfBounds = false;
    }
    public void resetToROS0_0_500()
    {
        updateROSTransformPosition(new Vector3(0, 0, 500));
    }
    public void updateROSTransformPosition(Vector3 newROSLocalPosition)
    {
        rosTransform.localPosition = newROSLocalPosition;
    }

    public void updateUnityTransformPosition(Vector3 newUnityLocalPosition)
    {
        ownTransform.localPosition = newUnityLocalPosition;
    }
    // Update is called once per frame
    void Update()
    {
        if (oldPosition != ownTransform.localPosition)
        {
            rosTransform.localPosition = transformer.testUnityLocal2ROS(ownTransform.localPosition);
            oldPosition = ownTransform.localPosition;
            if (isOutOfBounds)
            {

                meshRenderer.material = outOfRangeColorPrefab;
            }
            else
            {
                meshRenderer.material = selectedColorPrefab;
            }

        }
        else
        {
            if (oldRosTransformPosition != rosTransform.localPosition)
            {
                ownTransform.localPosition = transformer.testROS2UnityLocal(rosTransform.localPosition);
                oldPosition = ownTransform.localPosition;
            }
        }
        checkIfOutOfBounds();
        oldRosTransformPosition = rosTransform.localPosition;

        if (useROSTransform)
        {
            coordText.text = coordPrefix + rosTransform.localPosition.ToString("F2");
        }
        else
        {
            coordText.text = coordPrefix + ownTransform.localPosition.ToString("F2");

        }
    }

    private void checkIfOutOfBounds()
    {
        Vector3 rosTransformScaledInMeters = rosTransform.localPosition / 1000f;
      /*
        if(rosTransformScaledInMeters.localPosition.x < 0 ||
            //rosTransform.localPosition.x > 300 || 
            rosTransformScaledInMeters.localPosition.y < 0 ||
            // rosTransform.localPosition.y > 300 || 
            rosTransformScaledInMeters.localPosition.z < 0
            //rosTransform.localPosition.z > 300
            )
        {
            isOutOfBounds = true;

        }
        */
//        else
        //{
            float l1 = 0.135f;
            float l2 = 0.147f;
            float M_PI_2 = Mathf.PI / 2.0f;
            Vector3 lengthVector = new Vector3(rosTransformScaledInMeters.x, rosTransformScaledInMeters.y, 0);
            Vector3 toolOffset = new Vector3(0.069f, 0f, -0.06f);

            float l = lengthVector.magnitude - toolOffset.x;
            
            //it is 8.2cm from shoulder link to the robot base.
            //The inverse kinematics formula approximates that the shoulder_link is the world coordinate frame, not the base_link, hence we have to fix that.
            //Simular the hand joint is the target point, not the vacuum gripper. Thats why we have to subtract the toolOffset.
            float h = -(rosTransformScaledInMeters.z - toolOffset.z - 0.082f);

            //https://www.youtube.com/watch?v=IKOGwoJ2HLk
            float beta = -Mathf.Acos((Mathf.Pow(l, 2.0f) + Mathf.Pow(h, 2.0f) - Mathf.Pow(l1, 2.0f) - Mathf.Pow(l2, 2.0f)) / (2 * l1 * l2));
            float alpha = Mathf.Atan2(h, l) + Mathf.Atan2((l2 * Mathf.Sin(beta)), (l1 + l2 * Mathf.Cos(beta)));
            if (Mathf.Abs(Mathf.Atan2(rosTransformScaledInMeters.y, rosTransformScaledInMeters.x)) > (Mathf.PI * 130.0f/180.0f)  //("Unable to reach [%f,%f,%f]! Robot can only turn 130_ left/right!"
                ||
                l < 0 // "Unable to reach [%f,%f,%f]! Arm can not go so close to the robot!"
                ||
                alpha < (-M_PI_2 - 0.15) //"Unable to reach [%f,%f,%f]! Point is too close to the robot base!"
                || 
                float.IsNaN(alpha) || float.IsNaN(beta)
                )
            {
                isOutOfBounds = true;
            }

            else
            {
                isOutOfBounds = false;
            }
        //}
    }
}
