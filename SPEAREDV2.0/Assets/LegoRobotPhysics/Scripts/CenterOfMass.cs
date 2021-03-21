using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CenterOfMass : MonoBehaviour
{
    public Vector3 newCenterOfMass;
    public bool awake;
    public float gizmoSize = 10f;
    protected Rigidbody r;

    // Start is called before the first frame update
    void Start()
    {
        r = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        r.centerOfMass = newCenterOfMass;
        r.WakeUp();
        awake = !r.IsSleeping();
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position + transform.rotation * newCenterOfMass,gizmoSize);
    }
}
