using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PositionBall : DragOnFloor
{
    // Start is called before the first frame update
    void Start()
    {
        Vector3 v = this.transform.position;
        v.Set(round(v).x, round(v).y, round(v).z);
        transform.position = v;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
