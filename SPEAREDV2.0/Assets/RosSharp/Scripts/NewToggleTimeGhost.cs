using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewToggleTimeGhost : MonoBehaviour
{
    public TimeGhost timeGhost;
    public ExecuteNewGenProg executer;
    public bool DebugMode = true;
 

    public void toggleTimeGhosts()
    {
        timeGhost.ToggleGeisterzeit();
        this.ToggleDebuggingMode();
    }
    public void ToggleDebuggingMode()
    {
        DebugMode=executer.ToggleAllBreakPoints();
    }

    // Start is called before the first frame update
    void Start()
    {
        this.toggleTimeGhosts();
        //executer = this.gameObject.GetComponent<ExecuteGeneratedProgram>();
    }

    // Update is called once per frame
    void Update()
    {

    }
}
