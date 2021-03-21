using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public class PlayModeTests {
    [UnityTest]
    public IEnumerator ExampleSceneTest()
    {
        var activeScene = SceneManager.GetActiveScene();
        yield return SceneManager.LoadSceneAsync(1);
        // Use the Assert class to test conditions.
        var mainButton = GameObject.Find("MainButton");
        Assert.IsNotNull(mainButton, "Missing button " + "Main Button");
        var mainMenue = mainButton.transform.GetChild(1).gameObject;
        Assert.True(mainMenue != null, "MainMenu should exist");
        var button = mainButton.GetComponent<Button>();
        Assert.True(button, "MainButton should be a button");
        EventSystem.current.SetSelectedGameObject(mainButton);
        button.onClick.Invoke();
        Assert.True(mainMenue.activeSelf, "MainButton click should be activate Menu");
    }
    [Test]
    public void PlayModeTestsSimplePasses() {
        // Use the Assert class to test conditions.
        Assert.True(true);
    }
    // A UnityTest behaves like a coroutine in PlayMode
    // and allows you to yield null to skip a frame in EditMode
    [UnityTest]
    public IEnumerator PlayModeTestsWithEnumeratorPasses() {
        // Use the Assert class to test conditions.
        // yield to skip a frame
        Assert.True(true);
        yield return null;
        Assert.True(true);
    }
}
