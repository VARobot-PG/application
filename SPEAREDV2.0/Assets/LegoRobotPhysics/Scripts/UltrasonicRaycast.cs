using UnityEngine;

/* The UltrasonicRaycast takes care of the raycasting for the collisiondetection. It should be attached on the ultrasonic sensor model (or a child of it)
 * and it should have the same rotation and 'looking direction' as the sensor */
public class UltrasonicRaycast : MonoBehaviour
{
    private RaycastHit hit;                     /* The last raycast hit */
    private bool wasHit = false;                /* If the last Raycast hit anything */
    public float rayLength = 400.0f;            /* How far a ray should be casted */
    public bool enableLoggingMessages = false;  /* If debugguing messages should be logged */

    /* On every frame update, a Raycast is done and if it hits anything, the fact that a collision took place and the hit result are saved */
    void FixedUpdate()
    {
        Debug.DrawRay(this.transform.position, this.transform.forward * rayLength, Color.red, 0.0f);

        if(Physics.Raycast(this.transform.position, this.transform.forward, out hit, rayLength))
        {
            if(hit.collider.tag != "RobotPart")
            {
                wasHit = true;
                if (enableLoggingMessages) { Debug.Log("Seeing Gameobject " + hit.collider.name + " Distance: " + hit.distance); }
            }
        }
        else
        {
            wasHit = false;
        }
    }

    /* Returns if the last raycast had a collision with an object that is not tagged "RobotPart" closer than the allowedDistance */
    public bool facingThing(float allowedDistance)
    {
        if (wasHit) { 
            if (hit.collider.tag != "RobotPart")
            {
                if (enableLoggingMessages) { Debug.Log("FacingThing called, collision detected in distance " + hit.distance + " object is: " + hit.collider.name); }
                return hit.distance <= allowedDistance;
            }
            else
            {
                if (enableLoggingMessages) { Debug.Log("FacingThing called, no collision with apropriate object detected"); }
                return false;
            }
        }
        else
        {
            if (enableLoggingMessages) { Debug.Log("FacingThing called, no collision detected"); }
            return false;
        }
    }

    public void setEnableLoggingMessages(bool logging)
    {
        enableLoggingMessages = logging;
    }
}
