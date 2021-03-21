using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using System.Diagnostics;

public class SessionManagement
{

    [UnityTest]
    public IEnumerator NewSession()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // TS
        var newSessionButton = testReferenceManagerScript.arrReferences[30];
        Assert.IsNotNull(newSessionButton, "The new session button is missing.");
        var nButton = newSessionButton.GetComponent<Button>();
        Assert.True(nButton, "The new session button should be a button.");
        EventSystem.current.SetSelectedGameObject(newSessionButton);
        nButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT
        Scene activeScene = SceneManager.GetActiveScene();
        Assert.AreEqual(0, activeScene.buildIndex);
        // Assert.Equals("ChooseRobotScene", activeScene.name);

    }

    [UnityTest]
    public IEnumerator SaveAndLoadSession()
    {

        // var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE
        // PRE: click on main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // PRE: click on touch input
        var touchInputButton = testReferenceManagerScript.arrReferences[19];
        Assert.IsNotNull(touchInputButton, "The touch input button is missing.");
        var tButton = touchInputButton.GetComponent<Button>();
        Assert.True(tButton, "The touch input button should be a button.");
        EventSystem.current.SetSelectedGameObject(touchInputButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // TS
        // Add new movement command
        var newMovementButton = testReferenceManagerScript.arrReferences[31];
        Assert.IsNotNull(newMovementButton, "The new movement button is missing.");
        var nButton = newMovementButton.GetComponent<Button>();
        Assert.True(nButton, "The new movement button should be a button.");
        EventSystem.current.SetSelectedGameObject(newMovementButton);
        nButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // Save
        var saveSessionButton = testReferenceManagerScript.arrReferences[32];
        Assert.IsNotNull(saveSessionButton, "The save session button is missing.");
        var sButton = saveSessionButton.GetComponent<Button>();
        Assert.True(sButton, "The save session button should be a button.");
        EventSystem.current.SetSelectedGameObject(saveSessionButton);
        sButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // Add new movement command
        EventSystem.current.SetSelectedGameObject(newMovementButton);
        nButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // Load
        var loadSessionButton = testReferenceManagerScript.arrReferences[33];
        Assert.IsNotNull(loadSessionButton, "The load session button is missing.");
        var lButton = loadSessionButton.GetComponent<Button>();
        Assert.True(lButton, "The load session button should be a button.");
        EventSystem.current.SetSelectedGameObject(loadSessionButton);
        lButton.onClick.Invoke();
        yield return new WaitForSeconds(5);

        // OUT
        var robotCmds = testReferenceManagerScript.arrReferences[34];
        Assert.AreEqual(6, robotCmds.transform.childCount);

    }

}
