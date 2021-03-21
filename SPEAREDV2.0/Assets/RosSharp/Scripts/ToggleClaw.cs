using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class ToggleClaw : MonoBehaviour
{
   // public SetEndeffectorSuctionCup setEndeffectorSuctionCup;
    public bool clawUp = false;
    public MeshRenderer frontPlateToggleMeshRenderer;
    public Material toggleOnMaterial;
    public Material toggleOffMaterial;
    public bool sendCommandOn = false;
    public Transform toggleClawTransform;
    public TextMeshPro toggleText;
    public void toggleClaw()
    {
        clawUp = !clawUp;
       /* if (sendCommandOn)
            setEndeffectorSuctionCup.setEndEffectorSuctionCup(clawUp);    */
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (clawUp)
        {
            toggleClawTransform.name = "ToggleClaw_true";
            toggleText.text = "UP";                                             //text for what has to displayed in AR space (UP or DOWN)
            frontPlateToggleMeshRenderer.material = toggleOnMaterial;
        }
        else
        {
            toggleClawTransform.name = "ToggleClaw_false";
            toggleText.text = "DOWN";                                            //text for what has to displayed in AR space (UP or DOWN)
            frontPlateToggleMeshRenderer.material = toggleOffMaterial;
        }
    }
}
