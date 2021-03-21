using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using UnityEngine.UI;
using System.Text;  
using System.Text.RegularExpressions;

public class BasicUITests
{
    [UnityTest]
    public IEnumerator SpawnCubeTest() {           // both for spawning and despawning cubes
        // PRE
        yield return SceneManager.LoadSceneAsync(1);
        GameObject TestRefManInstance = GameObject.Find("TestReferenceManager");
        TestReferenceManager TestRefManInstanceC = TestRefManInstance.GetComponent<TestReferenceManager>();

        //Spawn button
        var SpawnBtn = TestRefManInstanceC.arrReferences[23];       //ModifiedUI/Canvas/Menu_OtherTools/Viewport/Content/SpawnCube
        Assert.NotNull(SpawnBtn, "Spawn Cube Button should exist");
        var SpawnBtnC = SpawnBtn.GetComponent<Button>();
        Assert.NotNull(SpawnBtnC, "Spawn Cube Button should be a button");

        //Despawn button
        var DeSpawnBtn = TestRefManInstanceC.arrReferences[25];     //ModifiedUI/Canvas/Menu_Control/Control_Despawn
        Assert.NotNull(DeSpawnBtn, "Despawn Cube Button should exist");
        var DeSpawnBtnC = DeSpawnBtn.GetComponent<Button>();      
        Assert.NotNull(DeSpawnBtnC, "Despawn Cube Button should be a button");

        var cubeParent = TestRefManInstanceC.arrReferences[24];   //DoBotContainer/scenario/GeneratedCubes
        Assert.NotNull(cubeParent, "Spawn Cube parent should exist");
        var cubeCount = cubeParent.transform.childCount;
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(15);

        // TC Spawn Cube
        EventSystem.current.SetSelectedGameObject(SpawnBtn);
        SpawnBtnC.onClick.Invoke();
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(30);
        // POST Spawn Cube
        var spawnCubeCount = cubeParent.transform.childCount;
        var increasedCubeCount = cubeCount+1;
        Assert.True(increasedCubeCount == spawnCubeCount, "Cube should be spawned"); 
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);

        // TC Despawn Cube
        LogAssert.Expect(LogType.Log,new Regex("Received DeleteModelResponse - statusMessage: DeleteModel: successfully deleted model success: True+"));
        EventSystem.current.SetSelectedGameObject(DeSpawnBtn);
        DeSpawnBtnC.onClick.Invoke();
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);
        // POST Despawn Cube
        var despawnCubeCount = cubeParent.transform.childCount;        
    }

    [UnityTest]
    public IEnumerator ProfilerTest() {         //test visibility of the Profiler
        // PRE
        yield return SceneManager.LoadSceneAsync(1);
        GameObject TestRefManInstance = GameObject.Find("TestReferenceManager");
        TestReferenceManager TestRefManInstanceC = TestRefManInstance.GetComponent<TestReferenceManager>();

        var ProfilerBtn = TestRefManInstanceC.arrReferences[26];   //ModifiedUI/Canvas/Menu_ToggleFeatures/Viewport/Content/Profiler

        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);   //wait to generate the dynamic game object profiler in the scene

        Assert.NotNull(ProfilerBtn, "Profiler Button should exist");
        var ProfilerBtnC = ProfilerBtn.GetComponent<Button>();
        Assert.NotNull(ProfilerBtnC, "Profiler Button should be a button");
        var mixedRealityPlaySpace = GameObject.Find("MixedRealityPlayspace");
        var diagnostics = mixedRealityPlaySpace.transform.GetChild(1);
        var ProfilerUI = diagnostics.transform.GetChild(0);
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);
        Assert.IsInstanceOf<Transform>(ProfilerUI);  //dynamically generated objects are of type Transform and not Game Object

        // TC Profiler disappearance
        EventSystem.current.SetSelectedGameObject(ProfilerBtn);
        ProfilerBtnC.onClick.Invoke();
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);

        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);

        // TC Profiler appearance
        ProfilerBtnC.onClick.Invoke();
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(10);
    }

    [UnityTest]
    public IEnumerator SphereResetTest() {          //TC for OtherTools -> Sphere Reset button
        // PRE
        yield return SceneManager.LoadSceneAsync(1);
        GameObject TestRefManInstance = GameObject.Find("TestReferenceManager");
        TestReferenceManager TestRefManInstanceC = TestRefManInstance.GetComponent<TestReferenceManager>();

        var SphereResetBtn = TestRefManInstanceC.arrReferences[27];   //ModifiedUI/Canvas/Menu_OtherTools/Viewport/Content/SphereReset

        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);   //wait to generate the dynamic game object in the scene

        Assert.NotNull(SphereResetBtn, "Sphere Reset Button should exist");
        var SphereResetBtnC = SphereResetBtn.GetComponent<Button>();
        Assert.NotNull(SphereResetBtnC, "Sphere Reset Button should be a button");

        var TargetSphere = TestRefManInstanceC.arrReferences[28];     //DoBotContainer/scenario/world/Dobot_Loader/TargetSphereContainer/RealTargetSphere
        Assert.NotNull(TargetSphere, "Target Sphere should exist");

        //TC: whether the target sphere gets reset
        var SphereCoord = TestRefManInstanceC.arrReferences[29];   //DoBotContainer/scenario/world/Dobot_Loader/TargetSphereContainer/RealTargetSphere/ROSCoords
        var SphereCoordC = SphereCoord.GetComponent<Transform>();

        EventSystem.current.SetSelectedGameObject(SphereResetBtn);
        SphereResetBtnC.onClick.Invoke();
        yield return new WaitForDomainReload();
        yield return new WaitForSeconds(8);   

        int checkX = (int)SphereCoordC.position.x;
        int checkY = (int)SphereCoordC.position.y;
        int checkZ = (int)SphereCoordC.position.z;

        //Assert.AreEqual(expectedValue, to be matched with, "String message in case of condition failure");
        //The values below are world values returned from transform in the scene for SphereCoordC.
        Assert.AreEqual(8, checkX, "Sphere is not reset in X position");
        Assert.AreEqual(0, checkY, "Sphere is not reset in Y position");
        Assert.AreEqual(-22, checkZ, "Sphere is not reset in Z position");     
    }

   

}

