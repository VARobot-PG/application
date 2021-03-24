using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleBackgroundMesh : MonoBehaviour
{
    public Material toggleOnMaterial;
    public Material toggleOffMaterial;
    public MeshRenderer targetButtonMesh;
    public bool toggleOn = false;
    public bool startToggleState = false;
    // Start is called before the first frame update
    void Start()
    {
        toggleOn = startToggleState;
    }

    public void toggle()
    {
        toggleOn = !toggleOn;
    }
    // Update is called once per frame
    void Update()
    {
        if (toggleOn)
        {
            targetButtonMesh.material = toggleOnMaterial;
        }
        else
        {
            targetButtonMesh.material = toggleOffMaterial;
        }
    }
}
