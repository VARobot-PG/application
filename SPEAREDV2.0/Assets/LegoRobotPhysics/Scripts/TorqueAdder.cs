using UnityEngine;

/* Adds torque to a GameObject with a RigidBody if attached to it. Should be attached to the robot wheels. */
public class TorqueAdder : MonoBehaviour
{
    public Vector3 torque = new Vector3(0,0,0); /* The torque force that should be added each frame */
    public bool enableLoggingMessages = false;  /* If debugguing messages should be logged */

    /* On every frame update, add the given torque to the wheel. */
    private void FixedUpdate()
    {
        this.gameObject.GetComponent<Rigidbody>().AddRelativeTorque(torque);
    }

    public void setX(float x)
    {
        torque = new Vector3(x, torque.y, torque.z);
    }

    public void setY(float y)
    {
        torque = new Vector3(torque.x, y, torque.z);
    }

    public void setMotor(float velocity, float force)
    {
        JointMotor motor = this.GetComponent<HingeJoint>().motor;
        if (motor.targetVelocity != velocity || motor.force != force)
        {
            motor.targetVelocity = velocity;
            motor.force = force;
            this.GetComponent<HingeJoint>().motor = motor;
        }
    }

    public void setZ(float z)
    {
        torque = new Vector3(torque.x, torque.y, z);
    }

    public void setEnableLoggingMessages(bool logging)
    {
        enableLoggingMessages = logging;
    }
}
