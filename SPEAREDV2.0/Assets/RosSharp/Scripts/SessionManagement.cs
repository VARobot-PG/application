using RosSharp.RosBridgeClient.MessageTypes.FileServer;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SessionManagement : MonoBehaviour
{
    public void NewSession()
    {
        SceneManager.LoadScene("ChooseRobotScene");
    }

    /*
    public void SaveSession()
    {
       
    }

    public void LoadSession()
    {
        
    }
    */

}
