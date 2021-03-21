using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class NewTouchInputCodingPanelControl : MonoBehaviour
{



    public Button addMovementButton;
    public Button addBruteMovementButton;
    public Button addClawButton;
    public Transform RobotCommandsWrapper;
    public GameObject RobotCommandLinePrefab;
    public ScrollRect ScrollView;
    public CodetoCanvasGenNew code2canvasInARSpace;
    private Program program;
    //public Button CodePanelAR;
    //private bool btnPressed = false;



    private void OnEnable()
    {
        
        this.program = serialize2ROSCode();
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
            var lineCtrl = line.GetComponent<NewLineOperations>();
            if (statement is MoveStatement1)
            {
                lineCtrl.ChangeCommand(0);
                lineCtrl.SetArg((statement as MoveStatement1).target.ToString());
            } 
            else if(statement is BruteMove)
            {   
                lineCtrl.ChangeCommand(1);
                lineCtrl.SetArg((statement as BruteMove).target.ToString());
            }
            else if (statement is ClawUp)
            {
                lineCtrl.ChangeCommand(2);
                // lineCtrl.SetArg((statement as ClawUp).isClawUp.ToString());
                if ((statement as ClawUp).isClawUp == true)
                {
                    lineCtrl.SetArg("up");
                }
                else
                {
                    lineCtrl.SetArg("down");
                }
            }
        }

    }
    private Program serialize2ROSCode()
    {

        Program p = new Program();
        p.statements = new List<Statement>();
        foreach (Transform child in this.RobotCommandsWrapper)
        {
            var lineCtrl = child.GetComponent<NewLineOperations>();
            p.statements.Add(lineCtrl.serialize2ROSCode());
        }
        return p;

    }

    public void makeAndRun()
    {
        code2canvasInARSpace.currentProgram = serialize2ROSCode();
        this.code2canvasInARSpace.makeAndRun();
    }
    public void simulateRun()
    {
        code2canvasInARSpace.currentProgram = serialize2ROSCode();
        this.code2canvasInARSpace.simulateRun();
    }

    // Start is called before the first frame update
    void Start()
    {

        addMovementButton.onClick.AddListener(() =>
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<NewLineOperations>();
            lo.ChangeCommand(0);
            ScrollView.verticalScrollbar.value = 0;
        });
        addBruteMovementButton.onClick.AddListener(() =>
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<NewLineOperations>();
            lo.ChangeCommand(1);
            ScrollView.verticalScrollbar.value = -1;
        });
        addClawButton.onClick.AddListener(() =>
        {
            GameObject go = Instantiate(RobotCommandLinePrefab, RobotCommandsWrapper);
            var lo = go.GetComponent<NewLineOperations>();
            lo.ChangeCommand(2);
            ScrollView.verticalScrollbar.value = -1;
        });
        /*CodePanelAR.onClick.AddListener(() =>
        {
            btnPressed = true;
        });*/
    }
}
