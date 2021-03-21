using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Urdf;
using System;

public class CollisionTester : MonoBehaviour
{
    public ExecuteGeneratedProgram currentExecuter;
    private bool sucker_is_on = true;
    private Collider myCollider;
    public GameObject sucker;
    // Start is called before the first frame update
    void Start()
    {
        myCollider = this.gameObject.GetComponent<Collider>();
    }

    // Update is called once per frame
    void Update()
    {
        //transform.
    }
    //this is the callback for the sucker_subscriber
    public void sucker_message_arrived(bool value)
    {
        sucker_is_on = value;

        //toggling the triger when the sucker is off, menaing any collison with
        //the dobot is relevant if the sucker is turned off as it can't be
        //intentional
        if (sucker_is_on == false)
        {
            if(myCollider == null)
            {
                myCollider = this.gameObject.GetComponent<Collider>();
            }
            myCollider.isTrigger = true;
        }
    }



    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag != "uncollidable")
        {


            string name = other.gameObject.name;
            Debug.Log("Collision detected" + name);

            //check if we are colliding with the dobot
            UrdfLink myUrdf;
            try
            {
                myUrdf = other.gameObject.GetComponentInParent<UrdfLink>();
            }
            catch (Exception e)
            {
                myUrdf = null;
            }
            if (myUrdf != null && myUrdf.gameObject.tag == "dobot")
            {
                //we are colliding with part of the dobot
                Debug.Log("crashed with dobot");
                //check if we are colliding with the sucker

                bool collision_is_relevant = false;
                if (sucker_is_on)
                {
                    //try to find the distance between the sucker and us and if it
                    //is  reasonably small we can assume that we collided with the
                    // and just ignore this collision
                    //other.gameObject.GetComponent<>();
                    Vector3 suckerPos = new Vector3();
                    if (other.gameObject.transform.tag == "sucker")
                    {
                        suckerPos = other.gameObject.transform.position;
                    }
                    else
                    {
                        if (sucker != null)
                        {
                            suckerPos = sucker.transform.position;
                        }
                    }
                    float dist = Vector3.Distance(this.gameObject.transform.position, suckerPos);
                    Debug.Log($"Distance frpm name to sucker is {dist}");
                    if (dist< 0.1f)
                    {
                        // we are very close to the sucker
                        // so let's do nothing
                        collision_is_relevant = false;
                    }
                    else
                    {
                        collision_is_relevant = true;
                    }
                }
                else
                {
                    //sucker is turned off so all collisons with the dobot are relevant
                    collision_is_relevant = true;
                }
                if (collision_is_relevant)
                {
                    currentExecuter.handleCollision(other, this.gameObject);
                }
                else
                {
                    Debug.Log("Collision detected between dobot and sucked object->" +
                        "ignoring it");
                }
            }
        }

    }
    private void OnCollisionEnter(Collision collision)
    {

        Transform opponent = collision.transform;
        if (collision.gameObject.tag == "detected_object")
        {
            foreach (ContactPoint contact in collision.contacts)
            {
                Debug.DrawRay(contact.point, contact.normal, Color.red);
            }
        }

    }

}
