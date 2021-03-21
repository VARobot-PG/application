using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereDrawer : MonoBehaviour
{
    public Transform nextCommandTargetROSCoords;
    public GameObject nextCommandTargetSphere;
    public List<GameObject> targetSpheres;
    public List<Transform> targetSphereTargetROSCoords;
    public int currentSphere = 0;
    // Start is called before the first frame update
    void Start()
    {
        clearTargets();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void clearTargets()
    {
        foreach (GameObject go in targetSpheres)
            go.SetActive(false);
        currentSphere = 0;
    }
    public void drawNewTargetSphereAtROSCoords(Vector3 targetROSCoords)
    {
        targetSpheres[currentSphere].SetActive(true);
        targetSphereTargetROSCoords[currentSphere].localPosition = targetROSCoords;
        currentSphere++;
    }
    public void drawSphereAtROSCoords(Vector3 targetROSCoords)
    {
        nextCommandTargetSphere.SetActive(true);
        nextCommandTargetROSCoords.localPosition = targetROSCoords;
    }
}
