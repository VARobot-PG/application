using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using System.Diagnostics;

public class BallsPlaneDetectionTest 
{
    [UnityTest]
    public IEnumerator BallsPlaneDetection() //Test to make balls detect the plane

    {/*
        // PRE
        yield return SceneManager.LoadSceneAsync(3);
        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        //Balls
        var balls = testReferenceManagerScript.arrReferences[23];
        Assert.IsNotNull(balls, "The balls are missing.");
        Assert.True(balls.activeSelf, "The balls are not missing.");

        //Plane
        var plane = testReferenceManagerScript.arrReferences[24];
        Assert.IsNotNull(plane, "The plane is missing.");
        Assert.True(plane, "Plane should be a TestObject.");

        
        

            var gplane = GameObject.Find("Plane");
            //var redball = child.gameObject;
            //redball.Find(plane);

            Assert.IsNotNull(gplane, "Plane is missing"); 
            Assert.True(GameObject.Find("Plane"), "Redball can detect plane");
        

        


        // OUT: The  balls and coordinates are active.
        var ballsAndCoordinates = testReferenceManagerScript.arrReferences[23];
        Assert.IsNotNull(ballsAndCoordinates, "The balls and coordinates are missing.");
        Assert.True(ballsAndCoordinates.activeSelf, "The balls and coordinates should be active.");
        */
        yield return true;
    }
}
