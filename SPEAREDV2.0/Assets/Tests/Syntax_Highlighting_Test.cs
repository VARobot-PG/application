﻿using UnityEngine;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public class Syntax_Highlighting_Test
{
    // A UnityTest behaves like a coroutine in Play Mode. In Edit Mode you can use
    // `yield return null;` to skip a frame.
    [UnityTest]
    public IEnumerator IsTextHighlightedUnitySimulationScene()
    {
        // Use the Assert class to test conditions.
        // Use yield to skip a frame.
        yield return SceneManager.LoadSceneAsync(1);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate Touch Input Panel - Click Make&Run Code Button 
        // Step 1: Click on the main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(2);

         // Step 2: Click on the Touch Input Panel button
        var touchInputButton = testReferenceManagerScript.arrReferences[19];
        Assert.IsNotNull(touchInputButton, "The Touch Input Panel button is missing.");
        var tButton = touchInputButton.GetComponent<Button>();
        Assert.True(tButton, "The Touch Input Panel button should be a button.");
        EventSystem.current.SetSelectedGameObject(touchInputButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(2);
         // Step 3: Click on the Make&Run Button
        var makeRunButton = testReferenceManagerScript.arrReferences[20];
        Assert.IsNotNull(makeRunButton, "The Make and Run code button is missing.");
        var mRButton = makeRunButton.GetComponent<Button>();
        Assert.True(mRButton, "The Make and Run code button should be a button.");
        EventSystem.current.SetSelectedGameObject(makeRunButton);
        mRButton.onClick.Invoke();

    
        // Syntax - Highlighting Test - Check the Dobot Code panel on Screen
        // Step 1: Activate Dobot Code Panel from the menu
        var dobotCodeButton = testReferenceManagerScript.arrReferences[21];
        Assert.IsNotNull(dobotCodeButton, "The Dobot Code button is missing.");
        var dCodeButton = dobotCodeButton.GetComponent<Button>();
        Assert.True(dCodeButton, "The Dobot Code button should be a button.");
        EventSystem.current.SetSelectedGameObject(dobotCodeButton);
        dCodeButton.onClick.Invoke();
        yield return new WaitForSeconds(2);

        // Step 2: Activate AR mode
        var inArButton = testReferenceManagerScript.arrReferences[22];
        Assert.IsNotNull(inArButton, "The AR button is missing.");
        var arButton = inArButton.GetComponent<Button>();
        Assert.True(arButton, "The AR button should be a button.");
        EventSystem.current.SetSelectedGameObject(inArButton);
        arButton.onClick.Invoke();
        yield return new WaitForSeconds(2);
        

        // Step 3: Capture the text from the panel
        // reference to the panel
        var onScreenDobotCodePanel = GameObject.Find("Description_Unformatted");
        Assert.IsNotNull(onScreenDobotCodePanel, "The on-screen dobot code panel is not found.");

         yield return new WaitForSeconds(4);
         

        yield return new WaitForSeconds(2);

        Debug.Log("before assert equals");
        
        var actual = onScreenDobotCodePanel.GetComponent<Text>().text;

        Debug.Log("check actual string: " + actual);
/*
        var expected = "<color=#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);\n"+
                                       "<color=#DDDB16>Move</color> (78 200 145);\n" +
                                       "<color=#DDDB16>Move</color> (133.4 174.3 0);\n" +
                                       "<color=#DDDB16>Move</color> (132.2 171.6 86.8);\n"+
                                       "<color=#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);\n";


        Debug.Log("check expected string: " + expected);
*/
        var i = 0;

        foreach (var line in actual.Split('\n'))
        {
            if(line.Contains("ToggleSuction")){
                Assert.IsTrue(line.Contains("<color=#4BC524>ToggleSuction</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }

            if(line.Contains("Move")){
                Assert.IsTrue(line.Contains("<color=#DDDB16>Move</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }

            if(line.Contains("True")){
                Assert.IsTrue(line.Contains("<color=#2C30D1>True</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }
            
            if(line.Contains("False")){
                Assert.IsTrue(line.Contains("<color=#E50E0E>False</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }
        }

      //  Assert.AreEqual(actual, expected);

        //Assert.AreEqual(onScreenDobotCodePanel.GetComponent<Text>().text,onScreenDobotCodePanel.GetComponent<Text>().text);
        //Assert.Equals(onScreenDobotCodePanel.GetComponent<Text>().text, "<color=#4BC524>ToggleSuction</color> (<color=#2C30D1>True</color>);\n< color =#DDDB16>Move</color> (78 200 145);\n< color =#DDDB16>Move</color> (133.4 174.3 0);\n< color =#DDDB16>Move</color> (132.2 171.6 86.8);\n< color =#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);");
        Debug.Log("After assert equals");
       
    }


    [UnityTest]
    public IEnumerator IsTextHighlightedNewRobotScene()
    {
        // Use the Assert class to test conditions.
        // Use yield to skip a frame.
        yield return SceneManager.LoadSceneAsync(2);

        // reference to button
        var testReferenceManager = GameObject.Find("TestReferenceManager");
        // reference to script
        var testReferenceManagerScript = testReferenceManager.GetComponent<TestReferenceManager>();

        // PRE: Activate Touch Input Panel - Click Make&Run Code Button 
        // Step 1: Click on the main button
        var mainButton = testReferenceManagerScript.arrReferences[0];
        Assert.IsNotNull(mainButton, "The main button is missing.");
        var mButton = mainButton.GetComponent<Button>();
        Assert.True(mButton, "The main button should be a button.");
        EventSystem.current.SetSelectedGameObject(mainButton);
        mButton.onClick.Invoke();
        yield return new WaitForSeconds(2);

        

        // Syntax - Highlighting Test - Check the Dobot Code panel on Screen
        // Step 1: Activate Dobot Code Panel from the menu

        // Step 2: Activate AR mode
        var inArButton = testReferenceManagerScript.arrReferences[22];
        Assert.IsNotNull(inArButton, "The AR button is missing.");
        var arButton = inArButton.GetComponent<Button>();
        Assert.True(arButton, "The AR button should be a button.");
        EventSystem.current.SetSelectedGameObject(inArButton);
        arButton.onClick.Invoke();
        yield return new WaitForSeconds(2);

        var dobotCodeButton = testReferenceManagerScript.arrReferences[21];
        Assert.IsNotNull(dobotCodeButton, "The Dobot Code button is missing.");
        var dCodeButton = dobotCodeButton.GetComponent<Button>();
        Assert.True(dCodeButton, "The Dobot Code button should be a button.");
        EventSystem.current.SetSelectedGameObject(dobotCodeButton);
        //dCodeButton.onClick.Invoke();
        arButton.onClick.Invoke();
        yield return new WaitForSeconds(2);
        
        // Step 2: Click on the Touch Input Panel button
        var touchInputButton = testReferenceManagerScript.arrReferences[19];
        Assert.IsNotNull(touchInputButton, "The Touch Input Panel button is missing.");
        var tButton = touchInputButton.GetComponent<Button>();
        Assert.True(tButton, "The Touch Input Panel button should be a button.");
        EventSystem.current.SetSelectedGameObject(touchInputButton);
        tButton.onClick.Invoke();
        yield return new WaitForSeconds(2);

        // Step 3: Click on the Make&Run Button
        var makeRunButton = testReferenceManagerScript.arrReferences[20];
        Assert.IsNotNull(makeRunButton, "The Make and Run code button is missing.");
        var mRButton = makeRunButton.GetComponent<Button>();
        Assert.True(mRButton, "The Make and Run code button should be a button.");
        EventSystem.current.SetSelectedGameObject(makeRunButton);
        mRButton.onClick.Invoke();
        yield return new WaitForSeconds(2);

        // Step 3: Capture the text from the panel
        // reference to the panel as Highlighted_text
        var onScreenDobotCodePanel = GameObject.Find("Highlighted_text");
        Assert.IsNotNull(onScreenDobotCodePanel, "The on-screen dobot code panel is not found.");

        Debug.Log("before assert equals");
        
        var actual = onScreenDobotCodePanel.GetComponent<Text>().text;

        Debug.Log("check actual string: " + actual);
/*
        var expected = "<color=#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);\n"+
                                       "<color=#DDDB16>Move</color> (78 200 145);\n" +
                                       "<color=#DDDB16>Move</color> (133.4 174.3 0);\n" +
                                       "<color=#DDDB16>Move</color> (132.2 171.6 86.8);\n"+
                                       "<color=#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);\n";


        Debug.Log("check expected string: " + expected);
*/
        var i = 0;

        foreach (var line in actual.Split('\n'))
        {
            if(line.Contains("ToggleSuction")){
                Assert.IsTrue(line.Contains("<color=#FF1801>ToggleSuction</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }

            if(line.Contains("Move")){
                Assert.IsTrue(line.Contains("<color=#55F30D>Move</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }
            
            if(line.Contains("Brute")){
                Assert.IsTrue(line.Contains("<color=#FF001C>Brute</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }

            if(line.Contains("True")){
                Assert.IsTrue(line.Contains("<color=#0053FF>True</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }
            
            if(line.Contains("False")){
                Assert.IsTrue(line.Contains("<color=#FFFFFF>False</color>"), "Match the color code in Syntax Highlighting test script to the Editor.");
            }
        }

      //  Assert.AreEqual(actual, expected);

        //Assert.AreEqual(onScreenDobotCodePanel.GetComponent<Text>().text,onScreenDobotCodePanel.GetComponent<Text>().text);
        //Assert.Equals(onScreenDobotCodePanel.GetComponent<Text>().text, "<color=#4BC524>ToggleSuction</color> (<color=#2C30D1>True</color>);\n< color =#DDDB16>Move</color> (78 200 145);\n< color =#DDDB16>Move</color> (133.4 174.3 0);\n< color =#DDDB16>Move</color> (132.2 171.6 86.8);\n< color =#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);");
        Debug.Log("After assert equals");
       
    }

}

