using UnityEngine;
using System.Collections;

// Bespoke using declarations

using UnityEngine.UI;
using System.Collections.Generic;

public class GUIManager : MonoBehaviour
{
    public static GUIManager Instance;

    public bool Debug = true;

    public Text DebugWindow;

    public List<GameObject> Panels;

    public GameObject SplashPanel;
    public GameObject ConnectionPanel;
  
    public 

  
  
    void Awake()
    {
        Instance = this;
    }

	void Start ()
    {
        LoadConnectionScreen();
    }
	
	void Update () {
	
	}

    
    public void UpdateRobotName(string name)
    {
        NXTManager.Instance.NXTName = name;
    }

    public void LoadConnectionScreen()
    {
        StartCoroutine(loopAndLoad());
    }

    IEnumerator loopAndLoad()
    {
        yield return new WaitForSeconds(3);

        SplashPanel.SetActive(false);

        ConnectionPanel.SetActive(true);
    }

    // public void LoadGameplayScreen()
    // {
    //     foreach(GameObject panel in Panels)
    //     {
    //         panel.SetActive(false);
    //     }

    //     GameplayPanel.SetActive(true);
    // }

    public void QuitApp()
    {
        Application.Quit();
    }

}
