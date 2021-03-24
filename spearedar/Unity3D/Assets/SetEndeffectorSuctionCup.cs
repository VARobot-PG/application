using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Dobot;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetEndeffectorSuctionCup : MonoBehaviour
{
    public RosConnector RosConnector;
    protected RosSocket RosSocket;
    public string ServiceEndpoint = "/Dobot_Loader/SetEndEffectorSuctionCup";
    public GameObject Sucker;
    public bool flickering_enabled = false;
    // Start is called before the first frame update
    void Start()
    {
        this.RosSocket = this.RosConnector.RosSocket;
    }


    // Update is called once per frame
    void Update()
    {

    }

    public void setEndEffectorSuctionCup(bool enabled)
    {

        byte enableCtrl = Convert.ToByte(enabled);
        byte suck = Convert.ToByte(enabled);
        bool isQueued = true;
        SetEndEffectorSuctionCupRequest request = new SetEndEffectorSuctionCupRequest(enableCtrl, suck, isQueued);
            string responseValue = RosSocket.CallService<SetEndEffectorSuctionCupRequest, SetEndEffectorSuctionCupResponse>(ServiceEndpoint, serviceResponseHandler, request);
        if (enabled)
        {
            flickering_enabled = true;
            StartCoroutine(Flicker(Sucker));
        }
        else
        {
            flickering_enabled = false;
        }
    }


    private IEnumerator Flicker(GameObject flickeringObject,float waitTime=0.25f)
    {
        while (flickering_enabled)
        {
            //set red
            List<Color> oldsColours = new List<Color>();
            foreach (Renderer renderer in flickeringObject.GetComponentsInChildren<Renderer>())
            {
                
                foreach (Material m in renderer.materials)
                {
                    oldsColours.Add(m.color);
                    Color newColour = new Color(m.color.r, m.color.g, m.color.b, m.color.a);
                    //initialize redshift
                    newColour.r += 0.4f;
                    newColour.g -= 0.4f;
                    newColour.b -= 0.4f;

                    if (newColour.r > 1.0f)
                    {
                        newColour.r = 1.0f;
                    }
                    if (newColour.b < .0f)
                    {
                        newColour.b = .0f;
                    }
                    if (newColour.g < .0f)
                    {
                        newColour.g = .0f;
                    }
                    m.color = newColour;
                }
            }
            yield return new WaitForSeconds(waitTime);
            int counter = 0;
            foreach (Renderer renderer in flickeringObject.GetComponentsInChildren<Renderer>())
            {

                foreach (Material m in renderer.materials)
                {
                    m.color = oldsColours[counter];
                    counter++;
                }
            }
            yield return new WaitForSeconds(waitTime);


        }
        yield return null;
    }

    private void serviceResponseHandler(SetEndEffectorSuctionCupResponse t)
    {

    }
}
