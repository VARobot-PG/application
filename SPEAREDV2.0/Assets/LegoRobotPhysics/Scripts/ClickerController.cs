using UnityEngine;

public class ClickerController : MonoBehaviour
{
    private bool pressed = false;               /* If the clicker was pressed by a GameObject colliding with it/touching it*/
    public float waitTime = 1f;               /* How long a GameObject must have contact with the clicker until it counts as pressed (to avoid triggering on short accidental collisions)*/
    float pressStart = -1;                      /* When the ongoing contact with a gameobject started, -1 of there is no contact*/
    public bool enableLoggingMessages = false;  /* If debugguing messages should be logged */

    /* When a new collision is detected, save the time of impact */
    private void OnCollisionEnter(Collision collision)
    {
        //Debug.Log("Collisiton start:" + collision.gameObject.name + " time: " + pressStart + "diff: " + (Time.fixedTime - pressStart));

        if (collision.gameObject.tag != "RobotPart")
        {
            pressStart = Time.fixedTime;
        }
    }

    /* When a collision keeps happening for longer than the waittime, set pressed flag to true */
    private void OnCollisionStay(Collision collision)
    {
        if (Time.fixedTime - pressStart > waitTime && pressStart > 0 && collision.gameObject.tag != "RobotPart")
        {
            //Debug.Log("Collisiton:" + collision.gameObject.name + " time: " + pressStart + "diff: " + (Time.fixedTime - pressStart));
            pressed = true;
        }
    }

    /* If a collision stops, set pressed flag to false, and pressstart to -1 since the clicker is no longer pressed*/
    private void OnCollisionExit(Collision collision)
    {
        //Debug.Log("Collisiton end:" + collision.gameObject.name + " time: " + pressStart + "diff: " + (Time.fixedTime - pressStart));

        if (collision.gameObject.tag != "RobotPart")
        {
            pressed = false;
            pressStart = -1;
        }
    }

    /* If the clicker is currently pressed (for longer than waittime)*/
    public bool isPressed()
    {
        return pressed;
    }

    public void setEnableLoggingMessages(bool logging)
    {
        enableLoggingMessages = logging;
    }
}
