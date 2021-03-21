using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ToggleSuctionCup : MonoBehaviour
{
    public SetEndeffectorSuctionCup setEndeffectorSuctionCup;
    public bool suctionOn = false;
    public MeshRenderer frontPlateToggleMeshRenderer;
    public Material toggleOnMaterial;
    public Material toggleOffMaterial;
    public bool sendCommandOn = false;
    public Transform toggleSuctionTransform;
    public TextMeshPro toggleText;
    public void toggleSuction()
    {
        suctionOn = !suctionOn;
        if(sendCommandOn)
            setEndeffectorSuctionCup.setEndEffectorSuctionCup(suctionOn);
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (suctionOn)
        {
            toggleSuctionTransform.name = "ToggleSuction_true";
            toggleText.text = "On";
            frontPlateToggleMeshRenderer.material = toggleOnMaterial;
        }
        else
        {
            toggleSuctionTransform.name = "ToggleSuction_false";
            toggleText.text = "Off";
            frontPlateToggleMeshRenderer.material = toggleOffMaterial;
        }
    }
}
