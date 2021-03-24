using UnityEngine;
using System.Collections;

using System;
using System.IO;
using System.Text;
using UnityEngine.UI;
using System.Collections.Generic;

using UnityEngine.SceneManagement;
using UnityEngine.UI;

using System.IO.Ports;

public class NXTManager : MonoBehaviour
{
    public static NXTManager Instance;
    private string scriptLocation;

    public static AndroidJavaClass UnityPlayer;
    public static AndroidJavaObject Activity;

    private AndroidJavaClass activityClass = null;
    private AndroidJavaObject activityContext = null;

    public AndroidJavaObject nxtPlugin = null;

    public string NXTName
    {
        get { return PlayerPrefs.GetString("Name", "NXT"); }
        set { PlayerPrefs.SetString("Name", value); }
    }

    public bool DynamicScriptLocation = false;
    public bool InitialiseOnStart = false;
    public bool autoEnableBT = false;

    public bool IsConnected = false;

    // Bluetooth

    public delegate void BluetoothUpdateEventHandler(bool state);
    public static event BluetoothUpdateEventHandler BluetoothUpdateEvent;


    void Awake()
    {
        Instance = this;
    }

    void Start()
    {
        scriptLocation = gameObject.name;

        Screen.sleepTimeout = SleepTimeout.NeverSleep;

        activityClass = new AndroidJavaClass("com.unity3d.player.UnityPlayer");
        activityContext = activityClass.GetStatic<AndroidJavaObject>("currentActivity");


        BluetoothUpdateEvent +=
            NXTManager_BluetoothUpdateEvent;


        if (GUIManager.Instance.DebugWindow != null)
            GUIManager.Instance.DebugWindow.text = "Found unity plugin!";

        using (AndroidJavaClass pluginClass = new AndroidJavaClass("com.defaultcompany.nxtbluetoothplugin.NXTManager"))
        {
            if (pluginClass != null)
            {
                if (GUIManager.Instance.DebugWindow != null)
                    GUIManager.Instance.DebugWindow.text = "Found nxt plugin!";

                nxtPlugin = pluginClass.CallStatic<AndroidJavaObject>("nxtManager");
                nxtPlugin.Call("setContext", activityContext);
            }
        }

        if (DynamicScriptLocation)
            nxtPlugin.CallStatic<string>("scriptLocation", scriptLocation);

        if (autoEnableBT)
            nxtPlugin.CallStatic<bool>("autoEnableBT", true);

        if (InitialiseOnStart)
            nxtPlugin.Call("initialisePlugin");

    }



    void Update()
    {

    }

    void OnDestroy()
    {

        BluetoothUpdateEvent -=
           NXTManager_BluetoothUpdateEvent;

    }


    private void NXTManager_BluetoothUpdateEvent(bool state)
    {
        // throw new System.NotImplementedException();
    }

    public void InitialisePlugin()
    {
        Debug.Log("In intitialization");
        Debug.Log(Application.persistentDataPath);
        if (nxtPlugin == null)
            return;

        nxtPlugin.SetStatic<string>("deviceName", NXTName);
        nxtPlugin.Call("initialisePlugin");
    }



    public void SendCodeToNxt()
    {
        //FILE I/O

        string myPath = Path.Combine(Application.persistentDataPath, "NxjFiles");

        nxtPlugin.Call("sendCodeToNXT", myPath);
    }

    public void InitialisePlugin(string nxtName)
    {
        if (nxtPlugin == null)
            return;

        NXTName = nxtName;
        InitialisePlugin();
    }


    void throwAlertUpdate(string data)
    {
        if (GUIManager.Instance.DebugWindow != null)
            GUIManager.Instance.DebugWindow.text = data;

        if (data.Equals("Connected to NXT"))
            SceneManager.LoadScene("NewRobotUnitySimulationScene");
    }

    void throwBluetoothConnection(string data)
    {
        switch (data)
        {
            case "true": IsConnected = true; break;
            case "false": IsConnected = false; break;
        }

        if (GUIManager.Instance.DebugWindow != null)
            GUIManager.Instance.DebugWindow.text = "Bluetooth Connected; " + IsConnected.ToString();

        if (BluetoothUpdateEvent != null)
            BluetoothUpdateEvent(IsConnected);
    }

    void throwBluetoothAlert(string data)
    {
        string[] input = data.Split(',');
    }

    // ===============================================================================================================================
    // Control the NXT Robot in NewRobotUnitySimulationScene 
    // ===============================================================================================================================

    public GameObject codePanel;
    public string nxtText;
    public GameObject nxtRobot;
    float robotResetX = 0f;
    float robotResetZ = 0f;
    float moveToX = 0f;
    float moveToZ = 0f;

    public void resetNXTPosition()
    {
        Debug.Log("UNITY NXT: Inside if loop --> reset NXT Button");
        robotResetX = moveToX;
        robotResetZ = moveToZ;
        Debug.Log("UNITY NXT: Robot has moved to: " + robotResetX + " " + robotResetZ);
        nxtPlugin.Call("resetNXTPos");
    }

    public void sendCoordsToNXT()
    {


       // robotResetX = nxtRobot.transform.position.x;
        //robotResetZ = nxtRobot.transform.position.z;

        Debug.Log("UNITY NXT position of robot: " + robotResetX + " " + robotResetZ);

        Debug.Log("in send coords to nxt");

        if (nxtPlugin == null)
        {
            Debug.Log("no ref to nxtplugin");
        }
        else
        {
            nxtText = codePanel.GetComponent<Text>().text;

            string[] commands = nxtText.Split('\n');

            foreach (string line in commands)
            {
                if (line.Contains("Move"))
                {
                    string innerLine = line.Substring(28);
                    Debug.Log("NXT Bluetooth: Innerline " + innerLine);

                    innerLine = innerLine.Replace(");", "");
                    string[] coords = innerLine.Split();

                    if (moveToX > robotResetX + (float.Parse(coords[0]) * 100))
                    {
                        nxtPlugin.Call("moveNXTBack", 20f, 0f);
                    }

                    moveToX = robotResetX + (float.Parse(coords[0]) * 100);
                    moveToZ = robotResetZ + (float.Parse(coords[2]) * 100);

                    Debug.Log("UNITY: Move to: " + moveToX + " " + moveToX);

                    nxtPlugin.Call("sendCoordsToNXT", moveToX, moveToZ);
                }

                if (line.Contains("Brute"))
                {
                    string innerLine = line.Substring(29);
                    Debug.Log("NXT Bluetooth: Innerline " + innerLine);

                    innerLine = innerLine.Replace(");", "");
                    string[] coords = innerLine.Split();

                    if (moveToX > robotResetX + (float.Parse(coords[0]) * 100))
                    {
                        nxtPlugin.Call("moveNXTBack", 20f, 0f);
                    }

                    moveToX = robotResetX + (float.Parse(coords[0]) * 100);
                    moveToZ = robotResetZ + (float.Parse(coords[2]) * 100);

                    Debug.Log("UNITY: Brute to: " + moveToX + " " + moveToZ);
                    nxtPlugin.Call("sendCoordsToNXT", moveToX, moveToZ);
                }

                if (line.Contains("Claw"))
                {
                    if (line.Contains("up"))
                    {
                        //THE NAMES ARE REVERSED IN THE ANDROID PLUGIN --> CAN BE INTERCHANGED LATER TO CORRECT UP AND DOWN
                        nxtPlugin.Call("sendClawCommandToNxt", "CLAW_UP");
                    }
                    if (line.Contains("down"))
                    {
                        nxtPlugin.Call("sendClawCommandToNxt", "CLAW_DOWN");
                    }
                }
            }
            nxtPlugin.Call("navigateNXTRobot");
        }
    }
}
