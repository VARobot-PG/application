using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TouchInputCodeingPanelControl : MonoBehaviour
{
    public GameObject NewRobot;

    public Button addMovementButton;
    public Button addSucctionButton;
    public Transform RobotCommandsWrapper;
    public GameObject RobotCommandLinePrefab;
    public ScrollRect ScrollView;
    public CodeToCanvasGenerator code2canvasInARSpace;
    private Program program;

    private void OnEnable()
    {
        this.program = code2canvasInARSpace.currentProgram;
        drawCurrentProgram();
    }
    private void OnDisable()
    {
        code2canvasInARSpace.currentProgram = serialize2ROSCode();
        code2canvasInARSpace.redraw();
    }
    private void drawCurrentProgram()
    {
        foreach(Transform child in this.RobotCommandsWrapper)
        {
            Destroy(child.gameObject);
        }
        foreach(var statement in this.program.statements)
        {
            var line = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lineCtrl = line.GetComponent<LineOperations>();
            if (statement is MoveStatement)
            {
                lineCtrl.ChangeCommand(0);
                lineCtrl.SetArg((statement as MoveStatement).target.ToString());
            } 
            else if(statement is ToggleSuction)
            {
                lineCtrl.ChangeCommand(1);
                lineCtrl.SetArg((statement as ToggleSuction).isSuctionEnabled.ToString());
            }
        }

    }
    private Program serialize2ROSCode()
    {

        Program p = new Program();
        p.statements = new List<Statement>();
        foreach (Transform child in this.RobotCommandsWrapper)
        {
            var lineCtrl = child.GetComponent<LineOperations>();
            p.statements.Add(lineCtrl.serialize2ROSCode());
        }
        return p;

    }

    public void makeAndRun()
    {
        code2canvasInARSpace.currentProgram = serialize2ROSCode();
        code2canvasInARSpace.fillProgramCodeUI();
        code2canvasInARSpace.redraw();

    }

    // Start is called before the first frame update
    void Start()
    {
        /*
        dropdownRobotCommands.GetComponent<Dropdown>().onValueChanged.AddListener(delegate
        {
            executeCmd(dropdownRobotCommands.GetComponent<Dropdown>());
        });
        */
        addMovementButton.onClick.AddListener(() =>
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<LineOperations>();
            lo.ChangeCommand(0);
            ScrollView.verticalScrollbar.value = 0;
        });
        addSucctionButton.onClick.AddListener(() =>
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<LineOperations>();
            lo.ChangeCommand(1);
            ScrollView.verticalScrollbar.value = -1;
        });
    }

    /*
    private void executeCmd(Dropdown dropdownRobotCommands)
    {
        if (dropdownRobotCommands.value == 0)
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<LineOperations>();
            lo.ChangeCommand(0);
            ScrollView.verticalScrollbar.value = 0;
        } else if (dropdownRobotCommands.value == 1)
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<LineOperations>();
            lo.ChangeCommand(1);
            ScrollView.verticalScrollbar.value = -1;
        }
    }
    */

    // Update is called once per frame
    void Update()
    {
        
    }
}
