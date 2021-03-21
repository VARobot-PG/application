using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandJointFixer : MonoBehaviour
{
    public HingeJoint shoulderJoint;
    public HingeJoint elbowJoint;
    public HingeJoint handJoint;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
 

    }
    void FixedUpdate()
    {
        /*
 //The hand joint is set manually so that the joint is always behaving like the true dobot magician arm.
 //Otherwise there would be a chance that the arm is not perfectly straight and this does not look good.
 double handRotation = -(shoulderJoint->Position(0) + elbowJoint->Position(0));
 handJoint->SetPosition(0, handRotation);
 */
        float handRotationX = -(shoulderJoint.transform.localRotation.x + elbowJoint.transform.localRotation.x);
        Quaternion handQuat = handJoint.transform.localRotation;
        handQuat.x = handRotationX;
        handJoint.transform.localRotation = handQuat;
    }
}
