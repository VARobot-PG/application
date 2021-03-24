using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class AllowedPointHandler : MonoBehaviour
{
    public Transform targetTransform;
    public GameObject prefabTargetSphere;
    public Transform targetRoot;
    public SetPTPCmdServiceClient client;
    public CoordinateTargetSelector coordinateTargetSelector;
    public List<Vector3> dobotCoords;
    public List<Vector3> dobotCoordsReachable;
    public Transform convertedTransformUnity;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void generateSpheres()
    {
        StartCoroutine("generateTestSpheres");
    }
    public void addSphere(Vector3 positionInROS)
    {
        GameObject newSphere = Instantiate(prefabTargetSphere);
        newSphere.transform.position = targetTransform.position;
        newSphere.transform.SetParent(targetRoot, true);
        foreach(Transform child in newSphere.transform)
        {
            child.localPosition = positionInROS;
        }
    }
    public List<Vector3> testSpheres()
    {
        
        Vector3 dobotCoords1 = new Vector3(215, 4, 145);
        Vector3 dobotCoords2 = new Vector3(215, 24, 0);
        Vector3 dobotCoords3 = new Vector3(0, 200, 241);
        Vector3 dobotCoord;
        dobotCoords = new List<Vector3>();
        dobotCoordsReachable = new List<Vector3>();

        //dobotCoords.Add(dobotCoords1);
        //for(int x = 0; x <= 200 ; x = x + 1)
        //{
        //    for(int y = 0; y <= 200; y = y + 1)
        //    {
        //        for (int z = 0; z <= 200; z = z  + 1)
        //        {
        //            if (x + y > 200 && z % 20 != 0 && x % 20 != 0 && y % 20 != 0)
        //            {
        //                dobotCoord = new Vector3(x, y, z);
        //                dobotCoords.Add(dobotCoord);
        //            }
        //        }
        //    }
        //}

        /*
        for (int x = 0; x <= 200; x = x + 1)
        {
            
            dobotCoord = new Vector3(x, 200, 100);
            dobotCoords.Add(dobotCoord);
           
        }
        for (int y = 0; y <= 200; y = y + 1)
        {

            dobotCoord = new Vector3(200, y, 0);
            dobotCoords.Add(dobotCoord);

        }
        for (int z = 0; z <= 200; z = z + 1)
        {

            dobotCoord = new Vector3(200, 100, z);
            dobotCoords.Add(dobotCoord);

        }
        */
        System.Random random = new System.Random(1);
        for (int i = 0; i < 2500; i++)
        {
            int x = random.Next(20, 215);
            int y = random.Next(20, 215);
            int z = random.Next(20, 215);

            // x < 40 -> y > 160
            bool xleq40 = x >= 40 || y > 160;
            // x < 60 -> y > 140
            bool xleq60 = x >= 60 || y > 140;
            // x <= 80 -> 120 < y <= 200;
            bool xleq80 = x > 80 || y > 120;
            // x < 100 -> y > 80;
            bool xleq100 = x >= 100 || y > 100;
            //  x <= 160 -> y > 40;
            bool xleq160 = x > 160 || y > 40;
            //  x <= 180 -> y > 20;
            bool xleq180 = x > 180 || y > 20;
            if (xleq40 && xleq60 && xleq80 && xleq100 && xleq160 && xleq180)
            {
                dobotCoord = new Vector3(x, y, z);

                dobotCoords.Add(dobotCoord);
            }
        }
        dobotCoords.Sort(Comparer<Vector3>.Create((vec1,vec2) => vec1.ToString().CompareTo(vec2.ToString())));
        return dobotCoords;

    }
    IEnumerator generateTestSpheres()
    {
        Vector3 dobotCoords1 = new Vector3(215, 4, 145);
        Vector3 dobotCoords2 = new Vector3(0, 200, 0);
        Vector3 dobotCoords3 = new Vector3(176, 24, 79);
        Vector3 dobotCoords4 = new Vector3(78, 200, 241);
        Vector3 dobotCoords5 = new Vector3(215, 24, 145);
        Vector3 dobotCoords6 = new Vector3(215, 200, 145);
        Vector3 dobotCoords7 = new Vector3(176, 200, 145);
        Vector3 dobotCoords8 = new Vector3(0, 200, 145);
        Vector3 dobotCoords9 = new Vector3(0, 200, 79);
        Vector3 dobotCoords10 = new Vector3(0, 200, 241);
        Vector3 dobotCoords11 = new Vector3(215, 24, 241);
        Vector3 dobotCoords12 = new Vector3(215, 24, 0);
        Vector3 dobotCoords13 = new Vector3(78, 200, 145);
        List<Vector3> dobotCoords = new List<Vector3>();
        //dobotCoords = testSpheres();
        RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest request = createPTPRequest(1, dobotCoords1);
        dobotCoords.Add(dobotCoords1);
        dobotCoords.Add(dobotCoords2);
        dobotCoords.Add(dobotCoords3);
        dobotCoords.Add(dobotCoords4);
        dobotCoords.Add(dobotCoords5);
        dobotCoords.Add(dobotCoords6);
        dobotCoords.Add(dobotCoords7);
        dobotCoords.Add(dobotCoords8);
        dobotCoords.Add(dobotCoords9);
        dobotCoords.Add(dobotCoords10);
        dobotCoords.Add(dobotCoords11);
        dobotCoords.Add(dobotCoords12);
        dobotCoords.Add(dobotCoords13);
        Vector3 oldTransform = targetTransform.position;
        foreach (Vector3 dobotCoord in dobotCoords)
        {
            oldTransform = targetTransform.position;
            request = createPTPRequest(1, dobotCoord);
            client.sendPTPCommand(request);
            yield return new WaitForSeconds(3.5f);
            if (Vector3.Distance(oldTransform,targetTransform.position) > 0.00005f)
            {
                dobotCoordsReachable.Add(dobotCoord);
                //addSphere(dobotCoord);
                addSphere(convertedTransformUnity.localPosition);
            }
        }
        coordinateTargetSelector.writeRosNodesToFile();


    }
    public RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest createPTPRequest(byte ptpMode, Vector3 target)
    {
        return new RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest(ptpMode, target.x, target.y, target.z, 0, true);
    }
    
}
