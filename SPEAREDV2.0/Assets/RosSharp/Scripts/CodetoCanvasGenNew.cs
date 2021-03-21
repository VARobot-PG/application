using Microsoft.MixedReality.Toolkit.Utilities;
using RosSharp.RosBridgeClient;
using System;
using System.Collections;
using System.Collections.Generic;
using RosSharp;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using System.Globalization;
using Assets.LejosTemplates;

public class CodetoCanvasGenNew : MonoBehaviour
{
    public Canvas canvas;
    public Transform rootCanvasUI;
    public Program currentProgram;
    public GameObject programCodeUnformatted;
    public GameObject programCodeFormatted;
    public NewICmdRepresentation commandUIRepresentation;
    GameObject rootCommandObject;
    public CoordinateTargetSelector coordinateTargetSelector;

    public GridObjectCollection baseObjectCollection;

    // public RosSharp.RosBridgeClient.RosConnector RosConnector;
    public ExecuteNewGenProg myExecutor;

    // /////////////////////////////////////////
    //public InputField inputField;
    public string stringCaught;
    //public GameObject iField;
    //public Text textToCatch;
    public GameObject onScreenDobotCodePanel;
    //public GameObject arDobotCodePanel;
    string displayCode;
    // ////////////////////////////////////////

    public MovementController movementController; //Add movementController of robot for simulation here

    // Start is called before the first frame update
    void Start()
    {
        RobotProgramStructure newrobotProgramStructure = new RobotProgramStructure();
        this.currentProgram = newrobotProgramStructure.sampleProgram;
        drawCurrentCode();
    }  
    public void redraw()
    {
        if(rootCanvasUI != null)
        {
            foreach (Transform child in rootCanvasUI)
            {
                if (child != null && child.name.StartsWith("Command"))
                {
                    child.gameObject.SetActive(false);
                    Destroy(child.gameObject);
                }

            }
            drawCurrentCode();
        }
    }
    public void convertCodeToProgram()
    {
        convertCodeToProgram(this.programCodeUnformatted.GetComponent<Text>().text);
        redraw();
    }
    public void convertCodeToProgram(string code)
    {
        Program program = new Program();
        List<Statement> statements = new List<Statement>();
        foreach (string line in code.Split('\n'))
        {
            if (line.Contains("Move"))
            {
                string strippedLine = line.Replace("Move", "");
                strippedLine = strippedLine.Trim(new char[] { '(', ')', ';' });
                strippedLine = strippedLine.Trim();
                string[] commandValues = strippedLine.Split(' ');
                statements.Add(statementFromString(commandValues));
            }
            //for converting code Brute as statement to program
            else if(line.Contains("Brute")){
                string strippedLine = line.Replace("Brute", "");
                strippedLine = strippedLine.Trim(new char[] { '(', ')', ';' });
                strippedLine = strippedLine.Trim();
                string[] commandValues = strippedLine.Split(' ');
                statements.Add(statementFromString(commandValues));
            }
            //for converting code ClawUp as statement to program
            else
            {
                if (line.Contains("Claw"))
                {
                    bool isClawUp = false;
                    string strippedLine = line.Replace("Claw", "");
                    strippedLine = strippedLine.Trim(new char[] { '(', ')', ';' });
                    strippedLine = strippedLine.Trim();
                    bool.TryParse(strippedLine, out isClawUp);
                    ClawUp toggle = new ClawUp();   //this is from the RobotProgramStructure file
                    toggle.isClawUp = isClawUp;
                    statements.Add(toggle);
                }


                /* we do not need toggle suction for this robot as of now
                if (line.Contains("ToggleSuction"))
                {
                    bool isSuctionEnabled = false;
                    string strippedLine = line.Replace("ToggleSuction", "");
                    strippedLine = strippedLine.Trim(new char[] { '(', ')', ';' });
                    strippedLine = strippedLine.Trim();
                    bool.TryParse(strippedLine, out isSuctionEnabled);
                    ToggleSuction1 toggle = new ToggleSuction1();
                    toggle.isSuctionEnabled = isSuctionEnabled;
                    statements.Add(toggle);
                }
                */
            }
        }
        program.statements = statements;
        this.currentProgram = program;
    }

    //creating statement from string for BruteMove
    private BruteMove brutestatementFromString(string[] commandValues)
    {
        if (commandValues.Length == 6) // movementType, x,y,z,r,isQueued
        {
            return brutestatementFromString(commandValues[0], commandValues[1], commandValues[2], commandValues[3], commandValues[4]);
        }
        else
        {
            if (commandValues.Length == 3) // x,y,z
            {
                return brutestatementFromString("1", commandValues[0], commandValues[1], commandValues[2], "0");
            }
        }
        return new BruteMoveToJ1();
    }
    private BruteMove brutestatementFromString(string movementType, string x, string y, string z, string r)
    {
        try
        {
            return brutestatementFromParsedString(int.Parse(movementType), float.Parse(x), float.Parse(y), float.Parse(z), float.Parse(r));
        }
        catch (Exception e)
        {
            return new BruteMoveToJ1();

        }
    }

    private MoveStatement1 statementFromString(string[] commandValues)
    {
        if (commandValues.Length == 6) // movementType, x,y,z,r,isQueued
        {
            return statementFromString(commandValues[0], commandValues[1], commandValues[2], commandValues[3], commandValues[4]);
        }
        else
        {
            if (commandValues.Length == 3) // x,y,z
            {
                return statementFromString("1", commandValues[0], commandValues[1], commandValues[2], "0");
            }
        }
        return new MoveToJ1();
    }
    private MoveStatement1 statementFromString(string movementType, string x, string y, string z, string r)
    {
        try
        {
            return statementFromParsedString(int.Parse(movementType), float.Parse(x), float.Parse(y), float.Parse(z), float.Parse(r));
        }
        catch (Exception e)
        {
            return new MoveToJ1();

        }
    }

    private MoveStatement1 statementFromParsedString(int movementType, float x, float y, float z, float r)
    {
        Vector4 target = new Vector4(x, y, z, r);
        MoveStatement1 moveStatement = new MoveToJ1(); // default
        switch (movementType)
        {
            case 0:
                moveStatement = new MoveToL1();
                break;
            case 1:
                moveStatement = new MoveToJ1();
                break;
            case 2:
                moveStatement = new JumpTo1();
                break;
            case 3:
                moveStatement = new MoveArc1();
                break;
            default:
                moveStatement = new MoveToJ1();
                break;
        }
        moveStatement.target = target;
        return moveStatement;
    }

     /*
            As of now, the example BruteMove types are below as for the move statement
    */
    private BruteMove brutestatementFromParsedString(int movementType, float x, float y, float z, float r)
    {
        Vector4 target = new Vector4(x, y, z, r);
        BruteMove bruteStatement = new BruteMoveToJ1(); // default as for move statement
        switch (movementType)
        {
            case 0:
                bruteStatement = new BruteMoveToL1();
                break;
            case 1:
                bruteStatement = new BruteMoveToJ1();
                break;
            case 2:
                bruteStatement = new BruteJumpTo1();
                break;
            case 3:
                bruteStatement = new BruteMoveArc1();
                break;
            default:
                bruteStatement = new BruteMoveToJ1();
                break;
        }
        bruteStatement.target = target;
        return bruteStatement;
    }

    // private void drawCurrentCode()
    public void drawCurrentCode()
    {

        GameObject dropdownObject;// = new GameObject();
        float distanceCounter = 0.5f;


        foreach (Statement statement in currentProgram.statements)
        {
            //debug
            Debug.Log("drawCurrentCode() :"+statement);
            dropdownObject = commandUIRepresentation.constructCommandUIComponent("Command" + (int)distanceCounter, statement);
            dropdownObject.transform.SetParent(rootCanvasUI, false);

            if (baseObjectCollection != null)
            {

            }
            else
            {
                dropdownObject.transform.localPosition += new Vector3(0, -distanceCounter * commandUIRepresentation.getCommandObjectBoundaries().height, 0);

            }
            distanceCounter += 1f;
        }
        foreach (MoveCodeCmdNew moveCodeCommand in rootCanvasUI.GetComponentsInChildren<MoveCodeCmdNew>())
        {
            moveCodeCommand.ctcGenerator = this;
        }
        foreach (DeleteCodeCmdNew deleteCodeCommand in rootCanvasUI.GetComponentsInChildren<DeleteCodeCmdNew>())
        {
            deleteCodeCommand.ctcGenerator = this;
        }
        foreach (AddCodeCmdNew addCodeCommand in rootCanvasUI.GetComponentsInChildren<AddCodeCmdNew>())
        {
            addCodeCommand.ctcGenerator = this;
        }
        foreach (NewSetBreakpointCmd setBreakpointCommand in rootCanvasUI.GetComponentsInChildren<NewSetBreakpointCmd>())
        {
            setBreakpointCommand.executeGeneratedProgram = myExecutor;
        }
        foreach (AddClawToggleCommand addClawToggleCommand in rootCanvasUI.GetComponentsInChildren<AddClawToggleCommand>())
        {
            addClawToggleCommand.ctcGenerator = this;
        }
        //skip the toggle suction command for new robot
        /*
        foreach (AddSuctionToggleCommand addSuctionToggleCommand in rootCanvasUI.GetComponentsInChildren<AddSuctionToggleCommand>())
        {
            addSuctionToggleCommand.ctcGenerator = this;
        } */
        if (baseObjectCollection != null)
        {
            baseObjectCollection.UpdateCollection();
        }

    }
    public Program generateProgramFromTransform()
    {
        return generateProgramFromTransform(this.rootCanvasUI);
    }
    private Program generateProgramFromTransform(Transform codeRootTransform)
    {
        Program program = new Program();
        List<Transform> transforms = new List<Transform>();
        codeRootTransform.GetComponentsInChildren<Transform>(transforms);
        List<Statement> statements = new List<Statement>();
        Statement statement;
        foreach (Transform commandTransform in transforms)
        {
            if (commandTransform.name.StartsWith("Command"))
            {
                
                statement = commandUIRepresentation.generateStatementFromUI(commandTransform.gameObject);
                //Debug 
                Debug.Log("generateProgramFromTransform :"+statement);  //problem is in the statement from commandUIRepresentation.generateStatementFromUI()
                statements.Add(statement);

            }
        }
        program.statements = statements;
        return program;
    }
    private String generateProgramStringFromProgram()
    {
        return generateProgramStringFromProgram(this.currentProgram);
    }
    private String generateProgramStringFromProgram(Program program)
    {
        String programString = "";
        foreach (Statement statement in program.statements)
        {
            //debug 
            Debug.Log("generateProgramStringFromProgram statement:"+statement);   
            if (statement.GetType().IsSubclassOf(typeof(MoveStatement1)))
            {
                RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest request;
                MoveStatement1 moveStatement = (MoveStatement1)statement;
                byte ptpMode_byte;
                byte.TryParse("" + moveStatement.movementValueForCanvas, out ptpMode_byte);
                float x = moveStatement.target.x;
                float y = moveStatement.target.y;
                float z = moveStatement.target.z;
                float r = 0f;
                request = new RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest(ptpMode_byte, x, y, z, r, true);
                programString += $"Move({request.x} {request.y} {request.z});\n";
            }
            //below is to generate Program string from Program for BruteMove. The string generated on UI will read 'Brute'
            else if(statement.GetType().IsSubclassOf(typeof(BruteMove))){
                RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest request;
                BruteMove bruteStatement = (BruteMove)statement;
                byte ptpMode_byte;
                byte.TryParse("" + bruteStatement.movementValueForCanvas, out ptpMode_byte);
                float x = bruteStatement.target.x;
                float y = bruteStatement.target.y;
                float z = bruteStatement.target.z;
                float r = 0f;
                request = new RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest(ptpMode_byte, x, y, z, r, true);
                programString += $"Brute({request.x} {request.y} {request.z});\n";
            }
            else
            {
                if (statement is ClawUp)
                {
                    ClawUp clawUp = (ClawUp)statement;
                    // programString += $"Claw({clawUp.isClawUp});\n";
                    if (clawUp.isClawUp == true)
                    {
                        programString += $"Claw({"up"});\n";
                    }
                    else
                    {
                        programString += $"Claw({"down"});\n";
                    }
                }

                /*
                if (statement is ToggleSuction)
                {
                    //  ToggleSuction toggleSuction = (ToggleSuction)statement;
                    //programString += $"ToggleSuction({toggleSuction.isSuctionEnabled});\n";   
                    programString += $"Move({0} {0} {0});\n";
                }
              */
            }  
        }
        //debug 
        Debug.Log("generateProgramStringFromProgram with co-ordinates :"+programString);
        return programString;
    }

    public void fillProgramCodeUI()
    {
        this.currentProgram = this.generateProgramFromTransform();

        /*
        //############DEBUGGING##########################
        myExecutor.connector = this.RosConnector;
        myExecutor.currentProgram = this.currentProgram;
        myExecutor.StartProgram(); 
        //############DEBUGGING########################
        */

        string output = highlightSyntax();

        this.programCodeUnformatted.GetComponent<Text>().text = output;
        onScreenDobotCodePanel.GetComponent<Text>().text = output;
        //string dummy = "test output";
        //this.programCodeUnformatted.GetComponent<Text>().text = dummy;
        //onScreenDobotCodePanel.GetComponent<Text>().text = dummy;

        /*int i = 0;
        int j = 0;
        string[] k = output.Split('\n');
        string[] check = new string[5] {"<color=#4BC524>ToggleSuction</color> (<color=#2C30D1>True</color>);",
                                       "<color=#DDDB16>Move</color> (78 200 145);",
                                       "<color=#DDDB16>Move</color> (133.4 174.3 0);",
                                       "<color=#DDDB16>Move</color> (132.2 171.6 86.8);",
                                       "<color=#4BC524>ToggleSuction</color> (<color=#E50E0E>False</color>);"};
        while(i<5 && j<5) { 
            Debug.Log(k[i++].Equals(check[j++]));
            Debug.Log("Value of i and j " + i +", "+j);  
        }  */

            /*       
         foreach(String line in k)
         {
             if (line.Equals(check[i++]))
             {
                 Debug.Log("From fill program UI-2: After checking if equal " + onScreenDobotCodePanel.GetComponent<Text>().text);
             }
             else
             {
                 Debug.Log("Something is wrong here in the if-else check:fillProgramUI, CodeToCanvasGenerator");
             }

         }  */

    }

    // ////////////////////////////////////////////////////

    //string colorToggleSuction;
    string colorMove;
    //BruteMove
    string colorBrute;
    //Claw command
    string colorClaw;
    string colorBoolTrue;
    string colorBoolFalse;

    public SyntaxColor syntaxColor;

    public string highlightSyntax()
    {

        //colorToggleSuction = "#" + syntaxColor.toggleSuctionColor[0].ToString("X2") + syntaxColor.toggleSuctionColor[1].ToString("X2") + syntaxColor.toggleSuctionColor[2].ToString("X2");
        colorMove = "#" + syntaxColor.moveColor[0].ToString("X2") + syntaxColor.moveColor[1].ToString("X2") + syntaxColor.moveColor[2].ToString("X2");
        colorBoolTrue = "#" + syntaxColor.trueColor[0].ToString("X2") + syntaxColor.trueColor[1].ToString("X2") + syntaxColor.trueColor[2].ToString("X2");
        colorBoolFalse = "#" + syntaxColor.falseColor[0].ToString("X2") + syntaxColor.falseColor[1].ToString("X2") + syntaxColor.falseColor[2].ToString("X2");

        //new color for BruteMove
        colorBrute = "#" + syntaxColor.bruteColor[0].ToString("X2") + syntaxColor.bruteColor[1].ToString("X2") + syntaxColor.bruteColor[2].ToString("X2");

        //new color for Claw command
        colorClaw = "#" + syntaxColor.clawColor[0].ToString("X2") + syntaxColor.clawColor[1].ToString("X2") + syntaxColor.clawColor[2].ToString("X2");

        /* Debug.Log("From Syntax color box: ");
        Debug.Log("Move: " + colorMove);
        Debug.Log("Move: " + bruteMove);
        
        Debug.Log("Inside the Syntax Highlighter for Android Screen Panel"); */
        stringCaught = generateProgramStringFromProgram();
        string intermediateOutput0 = "";
        string intermediateOutput1 = "";
        string intermediateOutput2 = "";
        string intermediateOutput3 = "";
        string intermediateOutput4 = "";
        string output = "";
        /*if (stringCaught.Contains("ToggleSuction"))
        {
            Debug.Log("string caught is toggle");
            intermediateOutput = stringCaught.Replace("ToggleSuction", "<color=" + colorToggleSuction + ">ToggleSuction</color> ");
            intermediateOutput = intermediateOutput.Replace("True)", "<color=" + colorBoolTrue + ">True</color>)");
            intermediateOutput = intermediateOutput.Replace("False)", "<color=" + colorBoolFalse + ">False</color>)");
        } */

        //highlighting without any condition
        intermediateOutput0 = stringCaught.Replace("Claw", "<color=" + colorClaw + ">Claw</color>");
        intermediateOutput1 = intermediateOutput0.Replace("True", "<color=" + colorBoolTrue + ">UP</color>");
        intermediateOutput2 = intermediateOutput1.Replace("False", "<color=" + colorBoolFalse + ">DOWN</color>");
        intermediateOutput3 = intermediateOutput2.Replace("Brute", "<color=" + colorBrute + ">Brute</color>");
        intermediateOutput4 = intermediateOutput3.Replace("Move", "<color=" + colorMove + ">Move</color>");

        /* if (stringCaught.Contains("Claw"))
         {
             //Debug
             Debug.Log("In syntax Claw");
             intermediateOutput0 = stringCaught.Replace("ClawUp", "<color=" + colorBoolTrue + ">ClawUp</color>");
             if (!stringCaught.Contains("Move") && !stringCaught.Contains("Brute"))
                 return intermediateOutput0;
         }
         if (stringCaught.Contains("Brute"))
         {
             //Debug
             Debug.Log("In syntax Brute");
             if (!stringCaught.Contains("Move") && !stringCaught.Contains("Claw"))  //string contains no move or claw command
             {
                 intermediateOutput1 = stringCaught.Replace("Brute", "<color=" + colorBrute + ">Brute</color>");
                 return intermediateOutput1;
             }
             else if (!stringCaught.Contains("Move") && stringCaught.Contains("Claw"))    //string contains no move but claw command
             {
                 intermediateOutput1 = intermediateOutput0.Replace("Brute", "<color=" + colorBrute + ">Brute</color>");
                 return intermediateOutput1;
             }
            else if (stringCaught.Contains("Move") && !stringCaught.Contains("Claw"))    //string contains move but no claw command
             {
                 intermediateOutput1 = stringCaught.Replace("Brute", "<color=" + colorBrute + ">Brute</color>");
             }
         }
         if (stringCaught.Contains("Move"))
         {
             //Debug
             Debug.Log("In syntax Move");
             if (!stringCaught.Contains("Brute") && !stringCaught.Contains("Claw"))   //string contains no brute but no claw command
             {
                 intermediateOutput2 = stringCaught.Replace("Move", "<color=" + colorMove + ">Move</color>");
                 return intermediateOutput2;
             }
             else if (!stringCaught.Contains("Brute") && stringCaught.Contains("Claw"))  //string contains no brute but claw command
             {
                 intermediateOutput2 = intermediateOutput0.Replace("Move", "<color=" + colorMove + ">Move</color>");
                 return intermediateOutput2;
             }
         }       */

        //Debug.Log("Check if String caught changed");
        //Debug.Log(output);

        output = intermediateOutput4;
        //Debug
        Debug.Log("string for syntax highlighting :"+output);
        return output;
    }
    // /////////////////////////////////////////////////////////////////
    // /////////////////////////////////////////////////////////////////
    public void makeAndRun()
    {
        this.moveNxtRobot(this.currentProgram);

        string output = this.highlightSyntax();

        this.programCodeUnformatted.GetComponent<Text>().text = output;

        GameObject onScreenCodePanel = GameObject.Find("Menu_NXTCode_Panel/Viewport/Content/AR_DoBotCode_Panel/Highlighted_text");
        try
        {
            onScreenCodePanel.GetComponent<Text>().text = output;
        }
        catch (System.NullReferenceException e)
        {
            Debug.Log("Code Panel not opened, NullReferenceException: " + e.ToString());
        }
    }
    public void simulateRun()
    {
        this.simulation(this.currentProgram);

        string output = this.highlightSyntax();

        this.programCodeUnformatted.GetComponent<Text>().text = output;

        GameObject onScreenCodePanel = GameObject.Find("Menu_NXTCode_Panel/Viewport/Content/AR_DoBotCode_Panel/Highlighted_text");
        try
        {
            onScreenCodePanel.GetComponent<Text>().text = output;
        }
        catch (System.NullReferenceException e)
        {
            Debug.Log("Code Panel not opened, NullReferenceException: " + e.ToString());
        }
    }

    /// <summary>
    /// SEND COMMANDS TO NXT ROBOT
    /// </summary>


    public void moveNxtRobot(Program program)
    {

        Program programToReturn = new Program();
        List<Statement> lejosStatements = new List<Statement>();

        foreach (Statement statement in program.statements)
        {

            LejosMoveStatement lejosMoveStatement = new LejosMoveStatement();

            if (statement is MoveStatement1 moveStatement)
            {
                float x = moveStatement.target.x;
                float z = moveStatement.target.z;

                lejosMoveStatement.x = (int)x;
                lejosMoveStatement.z = (int)z;

                lejosStatements.Add(lejosMoveStatement);
            }
            else if (statement is BruteMove bmoveStatement)
            {
                float x = bmoveStatement.target.x;
                float z = bmoveStatement.target.z;

                lejosMoveStatement.x = (int)x;
                lejosMoveStatement.z = (int)z;

                lejosStatements.Add(lejosMoveStatement);
            }

        }

        programToReturn.statements = lejosStatements;

        LejosMainCodeGen codeGen = new LejosMainCodeGen();
        codeGen.SetPackageName("test2");
        codeGen.SetProgramName("HalloWelt");
        codeGen.SetProgram(programToReturn);
        var compiler = GetComponent<LejosCompileRequester>();
        //StartCoroutine(compiler.RequestCompilation(codeGen.TransformText()));
        Debug.Log("Compiled");
    }


    /* Will feed the current program into the simulation, using the MovementController. */
    public void simulation(Program program)
    {
        Debug.Log("Program simulation called.");
        List<MovementController.states> stateList = new List<MovementController.states>();
        List<Vector3> values = new List<Vector3>();

        foreach (Statement statement in program.statements)
        {
            if (statement is MoveStatement1 moveStatement)
            {
                float x = moveStatement.target.x;
                float z = moveStatement.target.z;

                stateList.Add(MovementController.states.MOVE);
                values.Add(new Vector3(x, 0, z));
            }
            else if (statement is BruteMove bmoveStatement)
            {
                float x = bmoveStatement.target.x;
                float z = bmoveStatement.target.z;

                stateList.Add(MovementController.states.BRUTEMOVE);
                values.Add(new Vector3(x, 0, z));
            }
            else if (statement is ClawUp clawUpStatement)
            {
                if (clawUpStatement.isClawUp)
                {
                    stateList.Add(MovementController.states.RELEASE);
                    values.Add(new Vector3(0, 0, 0));
                }
                else
                {
                    stateList.Add(MovementController.states.GRAB);
                    values.Add(new Vector3(0, 0, 0));
                }
            }
        }
        movementController.simulateProgram(stateList, values);
    }
}

