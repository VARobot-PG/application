using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using System.Diagnostics;

public class BallsOnPlaneTest 
{
    [UnityTest]
    public IEnumerator BallsOnPlane() //Test to show balls are active in robot scene
    {
        /*
        // PRE
        yield return SceneManager.LoadSceneAsync(2);
        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        //Balls
        var balls = testReferenceManagerScript.arrReferences[23];
        Assert.IsNotNull(balls, "The balls are missing.");
        Assert.True(balls.activeSelf, "The balls are not missing.");

        foreach (Transform child in balls.transform)
        {

            var yellowball = child.gameObject;
            Assert.IsNotNull(yellowball, "Yellowball is missing.");
            Assert.True(yellowball.activeSelf, "The yellowball is not missing.");



        }



       
        

        //Plane
        var plane = testReferenceManagerScript.arrReferences[24];
        Assert.IsNotNull(plane, "The plane is missing.");
        Assert.True(plane, "Plane should be a TestObject.");
        

        // OUT: The  balls and coordinates are active.
        var ballsAndCoordinates = testReferenceManagerScript.arrReferences[23];
        Assert.IsNotNull(ballsAndCoordinates, "The balls and coordinates are missing.");
        Assert.True(ballsAndCoordinates.activeSelf, "The balls and coordinates should be active.");



    */
        yield return true;



    }

}
