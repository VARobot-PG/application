using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DragOnFloor : MonoBehaviour
{
    public float floorHeight = 0.05f;
    private bool moved = false;

    private void OnMouseDrag()
    {
        moved = true;
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        Plane plane=new Plane(Vector3.up, new Vector3(0, floorHeight, 0));        
        float distance;
        if (plane.Raycast(ray, out distance))
        {
            Vector3 v = ray.GetPoint(distance);
            v.Set(Mathf.Round(v.x * 10f) / 10f, Mathf.Round(v.y * 10f) / 10f, Mathf.Round(v.z * 10f) / 10f);
            transform.position = v;
        }
    }

    private void OnMouseUp()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        Plane plane = new Plane(Vector3.up, new Vector3(0, floorHeight, 0));
        float distance;
        if (plane.Raycast(ray, out distance))
        {
            Vector3 v = ray.GetPoint(distance);
            v.Set(Mathf.Round(v.x * 10f) / 10f, Mathf.Round(v.y * 10f) / 10f, Mathf.Round(v.z * 10f) / 10f);
            transform.position = v;
        }
        this.gameObject.SetActive(false);
    }
}
