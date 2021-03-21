using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class ChooseRobotControl : MonoBehaviour
{
    public void UnitySimulationScene()
    {
        SceneManager.LoadScene("UnitySimulationScene");
    }

    public void NewRobotUnitySimulationScene()
    {
        SceneManager.LoadScene("NXTBluetoothConnection");
    }

    public void GrabAndDropUnitySimulationScene()
    {
        SceneManager.LoadScene("GrabAndDropUnitySimulationScene");
    }
}
