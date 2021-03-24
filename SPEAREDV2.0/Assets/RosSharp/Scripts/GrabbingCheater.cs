using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GrabbingCheater : MonoBehaviour
{
    public float positionTolerance = 0.02f;
    public float waitTime = 0.5f;
    public Rigidbody ballBody;
    float firstEvent = -1;

    private void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "RobotPart" && other.GetComponentInParent<MovementController>().claw.clawposition == ClawControl.clawpositions.UP)
        {
            Debug.Log("Robot close to ball");
            firstEvent = Time.fixedTime;
            ballBody.mass = 100;
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (Time.fixedTime - firstEvent > waitTime && firstEvent > 0)
        {
            ballBody.mass = 0.000001f;
            //ballBody.transform.position = (GameObject.Find("legoNXTRobot_").transform.position + new Vector3(0, 0, 0.42f));
        }
    }

    private void OnTriggerExit(Collision collision)
    {
        //Debug.Log("Collisiton end:" + collision.gameObject.name + " time: " + pressStart + "diff: " + (Time.fixedTime - pressStart));

        if (collision.gameObject.tag == "RobotPart")
        {
            firstEvent = -1;
        }
    }


    /*private void OnCollisionEnter(Collision collision)
    {
        ballBody.mass = 100;
    }

    private void OnCollisionStay(Collision collision)
    {
        ballBody.mass = 0.000001f;
    }*/

    /*private void Update()
    {
        GameObject robot = GameObject.Find("legoNXTRobot");
        if (robot != null && similarPosition(this.transform.position, robot.transform.position))
        {
            Debug.Log("Robot close to ball");
        }
    }

    private bool similarPosition(Vector3 v1, Vector3 v2)
    {
        v1.y = 0;
        v2.y = 0;
        return System.Math.Abs((v1 - v2).magnitude) < positionTolerance;
    }*/
}
