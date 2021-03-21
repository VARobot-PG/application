using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.TestTools;


public class PlayModePhysicsEngineTests
{
    [UnityTest]
    public IEnumerator CheckConnectivity()
    {
        var webr = UnityWebRequest.Get("http://vm-eng-gazebo-ar-masters.cs.upb.de:9090/");
        yield return webr.SendWebRequest();
        Assert.True(webr.responseCode == 200, "Physics Engine should be accessible");
    }
}
