using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using System.Diagnostics;

public class ToggleDebugModeTests {

    /*
     * Elements in Array: 
     * MainButton, 
     * Toogle_Debug_Mode, 
     * Touch_MakeRun_TopLeft, 
     * Reset, 
     * Menu_Physics_Control, 
     * Menu_Control, 
     * Menu_DoBotCode_Panel, 
     * Menu_TouchInput, 
     * TargetSphereContainer, 
     * Menu_Pannel_VoiceCommands, 
     * Physics_Control, 
     * None, 
     * PhysicsFeaturesPanel, 
     * ARToggle (from PhysicsControl)
     */

    [UnityTest]
    public IEnumerator ActivateDebugMode()
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

        // TS: Activate debug mode - click debug mode button
        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        // tButton.onClick.AddListener(() => UnityEngine.Debug.Log(""));
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: "Make & Run Code" and "Reset" buttons are not active.
        var makeAndRunCodeButton = testReferenceManagerScript.arrReferences[2];
        Assert.IsNotNull(makeAndRunCodeButton, "The Make&Run Code button is missing.");
        var resetButton = testReferenceManagerScript.arrReferences[3];
        Assert.IsNotNull(resetButton, "The reset button is missing.");
        Assert.IsFalse(makeAndRunCodeButton.activeSelf, "Make&Run Code button should be hidden.");
        Assert.IsFalse(resetButton.activeSelf, "Reset button should be hidden.");

        // OUT: The menus "Physics Control", "Controls", "Dobot Code Panel", and "Touch Input" are active.
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        // UnityEngine.Debug.Log(physicsControlMenu.active);
        Assert.IsTrue(physicsControlMenu.activeSelf, "The physics control menu should be active.");
        Assert.IsTrue(controlsMenu.activeSelf, "The controls menu should be active.");
        Assert.IsTrue(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be active.");
        Assert.IsTrue(touchInputMenu.activeSelf, "The touch input menu should be active.");

        // OUT: The spheres and coordinates are active.
        var spheresAndCoordinates = testReferenceManagerScript.arrReferences[8];
        Assert.IsNotNull(spheresAndCoordinates, "The spheres and coordinates are missing.");
        Assert.True(spheresAndCoordinates.activeSelf, "The spheres and coordinates should be active.");

    }

    [UnityTest]
    public IEnumerator ActivateRunningMode()
    {
        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate debug mode - click main button - click debug mode button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS: Activate running mode - click debug mode button
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: "Make & Run Code" and "Reset" buttons are active.
        var makeAndRunCodeButton = testReferenceManagerScript.arrReferences[2];
        Assert.IsNotNull(makeAndRunCodeButton, "The Make&Run Code button is missing.");
        var resetButton = testReferenceManagerScript.arrReferences[3];
        Assert.IsNotNull(resetButton, "The reset button is missing.");
        Assert.True(makeAndRunCodeButton.activeSelf, "Make&Run Code button should be active.");
        Assert.True(resetButton.activeSelf, "Reset button should be active.");

        // OUT: All menus other than the main menu are hidden.
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        var voiceCommandsPanelMenu = testReferenceManagerScript.arrReferences[9];
        Assert.IsNotNull(voiceCommandsPanelMenu, "The voice commands panel menu is missing.");
        Assert.IsFalse(physicsControlMenu.activeSelf, "The physics control menu should be hidden.");
        Assert.IsFalse(controlsMenu.activeSelf, "The controls menu should be hidden.");
        Assert.IsFalse(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be hidden.");
        Assert.IsFalse(touchInputMenu.activeSelf, "The touch input menu should be hidden.");
        Assert.IsFalse(voiceCommandsPanelMenu.activeSelf, "The voice commands panel menu should be hidden.");

        // OUT: The spheres and coordinates are hidden.
        var spheresAndCoordinates = testReferenceManagerScript.arrReferences[8];
        Assert.IsNotNull(spheresAndCoordinates, "The spheres and coordinates are missing.");
        Assert.IsFalse(spheresAndCoordinates.activeSelf, "The spheres and coordinates should be hidden.");
    }

    [UnityTest]
    public IEnumerator ActivateDebugModeActiveMenus()
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

        // PRE: Open some menus - click on physics control button
        var physicsControlButton = testReferenceManagerScript.arrReferences[10];
        Assert.IsNotNull(physicsControlButton, "The physics control button is missing.");
        var pcButton = physicsControlButton.GetComponent<Button>();
        Assert.True(pcButton, "The physics control button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlButton);
        pcButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // yield return new WaitForSeconds(10);

        // Important to check!
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        Assert.True(physicsControlMenu.activeSelf, "The physics control menu should be active.");

        // TS: Activate debug mode - click debug mode button
        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // yield return new WaitForSeconds(20);

        // OUT: "Make & Run Code" and "Reset" buttons are not active.
        var makeAndRunCodeButton = testReferenceManagerScript.arrReferences[2];
        Assert.IsNotNull(makeAndRunCodeButton, "The Make&Run Code button is missing.");
        var resetButton = testReferenceManagerScript.arrReferences[3];
        Assert.IsNotNull(resetButton, "The reset button is missing.");
        Assert.IsFalse(makeAndRunCodeButton.activeSelf, "Make&Run Code button should be hidden.");
        Assert.IsFalse(resetButton.activeSelf, "Reset button should be hidden.");

        // OUT: The menus "Physics Control", "Controls", "Dobot Code Panel", and "Touch Input" are active.
        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        Assert.IsTrue(physicsControlMenu.activeSelf, "The physics control menu should be active.");
        Assert.IsTrue(controlsMenu.activeSelf, "The controls menu should be active.");
        Assert.IsTrue(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be active.");
        Assert.IsTrue(touchInputMenu.activeSelf, "The touch input menu should be active.");

        // OUT: The spheres and coordinates are active.
        var spheresAndCoordinates = testReferenceManagerScript.arrReferences[8];
        Assert.IsNotNull(spheresAndCoordinates, "The spheres and coordinates are missing.");
        Assert.True(spheresAndCoordinates.activeSelf, "The spheres and coordinates should be active.");

    }

    [UnityTest]
    public IEnumerator ActivateRunningModeInactiveMenus()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate debug mode - click main button - click debug mode button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: Hide some menus - click on physics control button
        var physicsControlButton = testReferenceManagerScript.arrReferences[10];
        Assert.IsNotNull(physicsControlButton, "The physics control button is missing.");
        var pcButton = physicsControlButton.GetComponent<Button>();
        Assert.True(pcButton, "The physics control button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlButton);
        pcButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // Important to check!
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        Assert.IsFalse(physicsControlMenu.activeSelf, "The physics control menu should be hidden.");

        // TS: Activate running mode - click debug mode button
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT: "Make & Run Code" and "Reset" buttons are active.
        var makeAndRunCodeButton = testReferenceManagerScript.arrReferences[2];
        Assert.IsNotNull(makeAndRunCodeButton, "The Make&Run Code button is missing.");
        var resetButton = testReferenceManagerScript.arrReferences[3];
        Assert.IsNotNull(resetButton, "The reset button is missing.");
        Assert.True(makeAndRunCodeButton.activeSelf, "Make&Run Code button should be active.");
        Assert.True(resetButton.activeSelf, "Reset button should be active.");

        // OUT: All menus other than the main menu are hidden.
        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        var voiceCommandsPanelMenu = testReferenceManagerScript.arrReferences[9];
        Assert.IsNotNull(voiceCommandsPanelMenu, "The voice commands panel menu is missing.");
        Assert.IsFalse(physicsControlMenu.activeSelf, "The physics control menu should be hidden.");
        Assert.IsFalse(controlsMenu.activeSelf, "The controls menu should be hidden.");
        Assert.IsFalse(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be hidden.");
        Assert.IsFalse(touchInputMenu.activeSelf, "The touch input menu should be hidden.");
        Assert.IsFalse(voiceCommandsPanelMenu.activeSelf, "The voice commands panel menu should be hidden.");

        // OUT: The spheres and coordinates are hidden.
        var spheresAndCoordinates = testReferenceManagerScript.arrReferences[8];
        Assert.IsNotNull(spheresAndCoordinates, "The spheres and coordinates are missing.");
        Assert.IsFalse(spheresAndCoordinates.activeSelf, "The spheres and coordinates should be hidden.");

    }

    [UnityTest]
    public IEnumerator ToggleDebugModeMenusSpace()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate debug mode - click main button - click debug mode button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        var toggleDebugModeButton = testReferenceManagerScript.arrReferences[1];
        Assert.IsNotNull(toggleDebugModeButton, "The debug mode button is missing.");
        var tButton = toggleDebugModeButton.GetComponent<Button>();
        Assert.True(tButton, "The debug mode button should be a button.");
        EventSystem.current.SetSelectedGameObject(toggleDebugModeButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: Open some menus in AR space - click on physics control->AR button
        var physicsControlARButton = testReferenceManagerScript.arrReferences[13];
        Assert.IsNotNull(physicsControlARButton, "The physics control AR button is missing.");
        var pcARButton = physicsControlARButton.GetComponent<Button>();
        Assert.True(pcARButton, "The physics control AR button should be a button.");
        EventSystem.current.SetSelectedGameObject(physicsControlARButton);
        pcARButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // Important to check!
        var physicsControlARMenu = testReferenceManagerScript.arrReferences[12];
        Assert.IsNotNull(physicsControlARMenu, "The physics control AR menu is missing.");
        Assert.True(physicsControlARMenu.activeSelf, "The physics control AR menu should be active.");

        // TS: Activate running mode - click debug mode button
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS: Activate debug mode - click debug mode button
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT:
        Assert.True(physicsControlARMenu.activeSelf, "The physics control AR menu should be active.");
        var physicsControlMenu = testReferenceManagerScript.arrReferences[4];
        Assert.IsNotNull(physicsControlMenu, "The physics control menu is missing.");
        Assert.IsFalse(physicsControlMenu.activeSelf, "The physics control menu should be hidden.");

        var controlsMenu = testReferenceManagerScript.arrReferences[5];
        Assert.IsNotNull(controlsMenu, "The controls menu is missing.");
        var dobotCodePanelMenu = testReferenceManagerScript.arrReferences[6];
        Assert.IsNotNull(dobotCodePanelMenu, "The dobot code panel menu is missing.");
        var touchInputMenu = testReferenceManagerScript.arrReferences[7];
        Assert.IsNotNull(touchInputMenu, "The touch input menu is missing.");
        Assert.IsTrue(controlsMenu.activeSelf, "The controls menu should be active.");
        Assert.IsTrue(dobotCodePanelMenu.activeSelf, "The dobot code panel menu should be active.");
        Assert.IsTrue(touchInputMenu.activeSelf, "The touch input menu should be active.");

        // OUT: "Make & Run Code" and "Reset" buttons are not active.
        var makeAndRunCodeButton = testReferenceManagerScript.arrReferences[2];
        Assert.IsNotNull(makeAndRunCodeButton, "The Make&Run Code button is missing.");
        var resetButton = testReferenceManagerScript.arrReferences[3];
        Assert.IsNotNull(resetButton, "The reset button is missing.");
        Assert.IsFalse(makeAndRunCodeButton.activeSelf, "Make&Run Code button should be hidden.");
        Assert.IsFalse(resetButton.activeSelf, "Reset button should be hidden.");

        // OUT: The spheres and coordinates are active.
        var spheresAndCoordinates = testReferenceManagerScript.arrReferences[8];
        Assert.IsNotNull(spheresAndCoordinates, "The spheres and coordinates are missing.");
        Assert.True(spheresAndCoordinates.activeSelf, "The spheres and coordinates should be active.");

    }

}
