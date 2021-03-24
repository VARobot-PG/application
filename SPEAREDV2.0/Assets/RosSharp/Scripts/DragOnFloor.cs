using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DragOnFloor : MonoBehaviour
{
    public float floorHeight = 0.05f;
    public Material purpleMat;
    public Material transparentMat;
    public bool selecting = true;

    private void OnMouseDrag()
    {
        Debug.Log("Onmousedrag, selecting:"+selecting);
        if (selecting)
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            Plane plane = new Plane(Vector3.up, new Vector3(0, floorHeight+0.025f, 0));
            float distance;
            if (plane.Raycast(ray, out distance))
            {
                Vector3 v = ray.GetPoint(distance);
                v.Set(round(v).x, round(v).y, round(v).z);
                //v.Set(Mathf.Round(v.x * 10f) / 10f, Mathf.Round(v.y * 1000f) / 1000f, Mathf.Round(v.z * 10f) / 10f);
                transform.position = v;
            }
        }
    }

    private void OnMouseUp()
    {
        if (selecting)
        {
            selecting = false;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            Plane plane = new Plane(Vector3.up, new Vector3(0, floorHeight+0.025f, 0));
            float distance;
            if (plane.Raycast(ray, out distance))
            {
                Vector3 v = ray.GetPoint(distance);
                v.Set(round(v).x, round(v).y, round(v).z);
                //v.Set(Mathf.Round(v.x * 10f) / 10f, Mathf.Round(v.y * 1000f) / 1000f, Mathf.Round(v.z * 10f) / 10f);
                transform.position = v;
            }
            this.GetComponent<MeshRenderer>().material = transparentMat;
            //this.gameObject.SetActive(false);
        }
    }

    protected Vector3 round(Vector3 v)
    {
        float x = Mathf.Round(v.x * 100f) / 100f;
        float y = Mathf.Round(v.y * 1000f) / 1000f;
        float z = Mathf.Round(v.z * 100f) / 100f;
        GameObject zero = GameObject.Find("AbsoluteZero");
        if(zero != null)
        {
            Debug.Log("AbsoluteZero at: " + zero.transform.position);
            x = x + (zero.transform.position.x - Mathf.Round(zero.transform.position.x * 100f) / 100f);
            y = y + (zero.transform.position.y - Mathf.Round(zero.transform.position.y * 1000f) / 1000f);
            z = z + (zero.transform.position.z - Mathf.Round(zero.transform.position.z * 100f) / 100f);
        }
        else
        {
            Debug.Log("AbsoluteZero not found");
        }
        Vector3 vector = new Vector3(x,y,z);
        return vector;
    }

    public void SetMaterialPurple()
    {
        this.GetComponent<MeshRenderer>().material = purpleMat;
    }
}
