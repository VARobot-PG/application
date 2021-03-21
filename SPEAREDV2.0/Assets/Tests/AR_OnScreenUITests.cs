using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using System.Diagnostics;

public class AR_OnScreenUITests {

    [UnityTest]
    public IEnumerator InARSpace()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate running mode - click on main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: Activate debug mode - click debug mode button
        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        // tButton.onClick.AddListener(() => UnityEngine.Debug.Log(""));
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS: Send all active menus to the AR space - click "In AR Space" button
        var inARSpaceButton = testReferenceManagerScript.arrReferences[14];
        Assert.IsNotNull(inARSpaceButton, "The In AR Space button is missing.");
        var arButton = inARSpaceButton.GetComponent<Button>();
        Assert.True(arButton, "The In AR Space button should be a button.");
        EventSystem.current.SetSelectedGameObject(inARSpaceButton);
        arButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: Menus in AR space are active.
        var physicsControlARMenu = testReferenceManagerScript.arrReferences[12];
        Assert.IsNotNull(physicsControlARMenu, "The physics control AR menu is missing.");
        var controlsARMenu = testReferenceManagerScript.arrReferences[15];
        Assert.IsNotNull(controlsARMenu, "The controls AR menu is missing.");
        var dobotCodePanelARMenu = testReferenceManagerScript.arrReferences[16];
        Assert.IsNotNull(dobotCodePanelARMenu, "The dobot code panel AR menu is missing.");
        var touchInputARMenu = testReferenceManagerScript.arrReferences[17];
        Assert.IsNotNull(touchInputARMenu, "The touch input AR menu is missing.");
        Assert.IsTrue(physicsControlARMenu.activeSelf, "The physics control AR menu should be active.");
        Assert.IsTrue(controlsARMenu.activeSelf, "The controls AR menu should be active.");
        Assert.IsTrue(dobotCodePanelARMenu.activeSelf, "The dobot code panel AR menu should be active.");
        Assert.IsTrue(touchInputARMenu.activeSelf, "The touch input AR menu should be active.");

        // OUT: Menus on screen are not active.
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        // var voiceCommandsPanelMenu = testReferenceManagerScript.arrReferences[9];
        // Assert.IsNotNull(voiceCommandsPanelMenu, "The voice commands panel menu is missing.");
        Assert.IsFalse(physicsControlMenu.activeSelf, "The physics control menu should be hidden.");
        Assert.IsFalse(controlsMenu.activeSelf, "The controls menu should be hidden.");
        Assert.IsFalse(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be hidden.");
        Assert.IsFalse(touchInputMenu.activeSelf, "The touch input menu should be hidden.");
        // Assert.IsFalse(voiceCommandsPanelMenu.activeSelf, "The voice commands panel menu should be hidden.");

    }

    [UnityTest]
    public IEnumerator OnScreen()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate running mode - click on main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: Activate debug mode - click debug mode button
        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        // tButton.onClick.AddListener(() => UnityEngine.Debug.Log(""));
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: Send all active menus to the AR space - click "In AR Space" button
        var inARSpaceButton = testReferenceManagerScript.arrReferences[14];
        Assert.IsNotNull(inARSpaceButton, "The In AR Space button is missing.");
        var arButton = inARSpaceButton.GetComponent<Button>();
        Assert.True(arButton, "The In AR Space button should be a button.");
        EventSystem.current.SetSelectedGameObject(inARSpaceButton);
        arButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS: Send all active menus to screen - click "On Screen" button
        var onScreenButton = testReferenceManagerScript.arrReferences[18];
        Assert.IsNotNull(onScreenButton, "The On Screen button is missing.");
        var osButton = onScreenButton.GetComponent<Button>();
        Assert.True(osButton, "The On Screen button should be a button.");
        EventSystem.current.SetSelectedGameObject(onScreenButton);
        osButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: Menus in AR space are not active.
        var physicsControlARMenu = testReferenceManagerScript.arrReferences[12];
        Assert.IsNotNull(physicsControlARMenu, "The physics control AR menu is missing.");
        var controlsARMenu = testReferenceManagerScript.arrReferences[15];
        Assert.IsNotNull(controlsARMenu, "The controls AR menu is missing.");
        var dobotCodePanelARMenu = testReferenceManagerScript.arrReferences[16];
        Assert.IsNotNull(dobotCodePanelARMenu, "The dobot code panel AR menu is missing.");
        var touchInputARMenu = testReferenceManagerScript.arrReferences[17];
        Assert.IsNotNull(touchInputARMenu, "The touch input AR menu is missing.");
        Assert.IsFalse(physicsControlARMenu.activeSelf, "The physics control AR menu should be hidden.");
        Assert.IsFalse(controlsARMenu.activeSelf, "The controls AR menu should be hidden.");
        Assert.IsFalse(dobotCodePanelARMenu.activeSelf, "The dobot code panel AR menu should be hidden.");
        Assert.IsFalse(touchInputARMenu.activeSelf, "The touch input AR menu should be hidden.");

        // OUT: Menus on screen are active.
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        // var voiceCommandsPanelMenu = testReferenceManagerScript.arrReferences[9];
        // Assert.IsNotNull(voiceCommandsPanelMenu, "The voice commands panel menu is missing.");
        Assert.IsTrue(physicsControlMenu.activeSelf, "The physics control menu should be active.");
        Assert.IsTrue(controlsMenu.activeSelf, "The controls menu should be active.");
        Assert.IsTrue(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be active.");
        Assert.IsTrue(touchInputMenu.activeSelf, "The touch input menu should be active.");
        // Assert.IsFalse(voiceCommandsPanelMenu.activeSelf, "The voice commands panel menu should be hidden.");

        // TODO: Menu_OtherTools, Menu_ToggleFeatures

    }

    [UnityTest]
    public IEnumerator MenuToAR()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate running mode - click on main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS: Activate Physics Control Menu in AR Space
        var physicsControlButton = testReferenceManagerScript.arrReferences[10];
        Assert.IsNotNull(physicsControlButton, "The physics control button is missing.");
        var pcButton = physicsControlButton.GetComponent<Button>();
        Assert.True(pcButton, "The physics control button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlButton);
        pcButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        var physicsControlARButton = testReferenceManagerScript.arrReferences[13];
        Assert.IsNotNull(physicsControlARButton, "The physics control AR button is missing.");
        var pcARButton = physicsControlARButton.GetComponent<Button>();
        Assert.True(pcARButton, "The physics control AR button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlARButton);
        pcARButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: Physics Control AR Menu is active.
        var physicsControlARMenu = testReferenceManagerScript.arrReferences[12];
        Assert.IsNotNull(physicsControlARMenu, "The physics control AR menu is missing.");
        Assert.True(physicsControlARMenu.activeSelf, "The physics control AR menu should be active.");

        // OUT: Physics Control Menu on Screen is not active.
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        Assert.IsFalse(physicsControlMenu.activeSelf, "The physics control menu should be hidden.");

    }

    [UnityTest]
    public IEnumerator MenuToScreen()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate running mode - click on main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: Activate Physics Control Menu in AR Space
        var physicsControlButton = testReferenceManagerScript.arrReferences[10];
        Assert.IsNotNull(physicsControlButton, "The physics control button is missing.");
        var pcButton = physicsControlButton.GetComponent<Button>();
        Assert.True(pcButton, "The physics control button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlButton);
        pcButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        var physicsControlARButton = testReferenceManagerScript.arrReferences[13];
        Assert.IsNotNull(physicsControlARButton, "The physics control AR button is missing.");
        var pcARButton = physicsControlARButton.GetComponent<Button>();
        Assert.True(pcARButton, "The physics control AR button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlARButton);
        pcARButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS: Send Physics Control Menu to Screen
        pcARButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: Physics Control AR Menu is not active.
        var physicsControlARMenu = testReferenceManagerScript.arrReferences[12];
        Assert.IsNotNull(physicsControlARMenu, "The physics control AR menu is missing.");
        Assert.IsFalse(physicsControlARMenu.activeSelf, "The physics control AR menu should be hidden.");

        // OUT: Physics Control Menu on Screen is active.
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        Assert.IsTrue(physicsControlMenu.activeSelf, "The physics control menu should be active.");

    }

}
