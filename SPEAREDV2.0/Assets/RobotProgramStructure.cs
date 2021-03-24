using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class RobotProgramStructure : MonoBehaviour
{

    public Program sampleProgram;
    public Transform nxtTransform;
    /*
     * // Example ROS Cube 132.073, 732.953, 63.937
     * ToggleSuction(True);
SetPTPCmdRequest(1,80,2,156,5,59,8,0,True);
SetPTPCmdRequest(1,133,4,174,3,0,0,True);
SetPTPCmdRequest(1,132,2,171,6,86,8,0,True);
ToggleSuction(False);
*/
    /*
   * // Example ROS Cube 132.073, 732.953, 63.937
   * ToggleSuction(True);
SetPTPCmdRequest(1,80,2,156,5,59,8,0,True);
SetPTPCmdRequest(1,133,4,174,3,0,0,True);
SetPTPCmdRequest(1,132,2,171,6,86,8,0,True);
ToggleSuction(False);
*/
    public RobotProgramStructure()
    {
        this.sampleProgram = new Program();
        List<Statement> statements = new List<Statement>();
        Vector3 dobotCoords1 = new Vector3(215, 4, 145);
        Vector3 dobotCoords2 = new Vector3(0, 200, 0); 
        Vector3 dobotCoords3 = new Vector3(176, 24, 79); 
        Vector3 dobotCoords4 = new Vector3(78, 200, 241);
        Vector3 dobotCoords5 = new Vector3(215, 24, 145);
        Vector3 dobotCoords6 = new Vector3(215, 200, 145);
        Vector3 dobotCoords7 = new Vector3(176, 200, 145);
        Vector3 dobotCoords8 = new Vector3(0, 200, 145);
        Vector3 dobotCoords9 = new Vector3(0, 200, 79);
        Vector3 dobotCoords10 = new Vector3(0, 200, 241);
        Vector3 dobotCoords11 = new Vector3(215, 24, 241);
        Vector3 dobotCoords12 = new Vector3(215, 24, 0);
        Vector3 dobotCoords13 = new Vector3(78, 200, 145);

        Vector3 dobotCoordsAboveCube = new Vector3(78f, 200f, 145f);
        Vector3 dobotCoordsCube = new Vector3(133.4f, 174.3f, 0f);
        Vector3 dobotCoordsAboveCube2 = new Vector3(132.2f, 171.6f, 86.8f);
        Vector3 nxtCoordsBrute = new Vector3(78f, 0f, 200f);

        
        //decide which scene is currently active
        Scene currentScene = SceneManager.GetActiveScene ();
        string sceneName = currentScene.name;
        //putting the below commands to be spawned inside conditions
        if(sceneName == "UnitySimulationScene")
        {
            ToggleSuction toggleSuction = new ToggleSuction();
            toggleSuction.isSuctionEnabled = true;
            statements.Add(toggleSuction); 
            statements = newMovementCommand(dobotCoordsAboveCube, new MoveToJ(), statements);
            statements = newMovementCommand(dobotCoordsCube, new MoveToJ(), statements);
        
            statements = newMovementCommand(dobotCoordsAboveCube2, new MoveToJ(), statements);
            toggleSuction = new ToggleSuction();
            toggleSuction.isSuctionEnabled = false;
            statements.Add(toggleSuction);  
        }
      
        /*
        else
        {
            //Generate a ClawUp command below
            ClawUp clawUp = new ClawUp();
            Debug.Log("ClawUp is being tried to create!");

            statements = newMovementCommand(new Vector3(0.6f, 0f, 0f), new MoveToJ(), statements);
            clawUp.isClawUp = true;
            statements.Add(clawUp);
            statements = newMovementCommand(new Vector3(1.1f, 0f, 0f), new MoveToJ(), statements);
            //Generate a ClawUp command below
            clawUp = new ClawUp();
            Debug.Log("ClawUp is being tried to create!");
            clawUp.isClawUp = false;
            statements.Add(clawUp);
            statements = newMovementCommand(new Vector3(0.9f, 0f, -0.4f), new MoveToJ(), statements);

        }
        */
        /*
         * not possible
         *  0 24 0
         * 
         * 
         * 
         */
        /*
       statements = newMovementCommand(dobotCoords10, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords5, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords3, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords8, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords13, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords1, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords6, new MoveToJ(), statements);
       statements = newMovementCommand(dobotCoords12, new MoveToJ(), statements);
       */
        //statements = newCommand(dobotCoords2, new MoveToJ(), statements);
        //statements = newCommand(dobotCoords9, new MoveToJ(), statements);
        //statements = newCommand(dobotCoords4, new MoveToJ(), statements);
        //statements = newCommand(dobotCoords7, new MoveToJ(), statements);
        //statements = newCommand(dobotCoords11, new MoveToJ(), statements);

        this.sampleProgram.statements = statements;
    }
    private List<Statement> newMovementCommand(Vector3 target, MoveStatement statement, List<Statement> statements)
    {
        statement.target = target;
        statements.Add(statement);
        return statements;
    }

    //NewMove command below
    private List<Statement> newMovementCommand(Vector3 target, MoveStatement1 statement, List<Statement> statements)
    {
        statement.target = target;
        statements.Add(statement);
        return statements;
    }

    //BruteMove command below
    private List<Statement> bruteMoveCommand(Vector3 target, BruteMove bruteStatement, List<Statement> statements)
    {   //Brute move command is constructed into a statement here 
        bruteStatement.target = target;
        statements.Add(bruteStatement);
        return statements;
    }

}
public class Program
{
    public List<Statement> statements;
    public List<string> stringStatements;
}
public abstract class Statement
{
    
    
}

public class ToggleSuction : Statement
{
    public bool isSuctionEnabled = false;

}
public abstract class MoveStatement : Statement
{
    public Vector3 target;
    public int movementValueForCanvas = -1;
    public RosSharp.RosBridgeClient.SetPTPCmdServiceClient serviceClient;



    public RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest SendCommandToRos()
    {
        byte ptpMode_byte;
        byte.TryParse("" + movementValueForCanvas, out ptpMode_byte);
        RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest request =
            new RosSharp.RosBridgeClient.MessageTypes.Dobot.
            SetPTPCmdRequest(ptpMode_byte, target.x, target.y,
            target.z, 0, true);
        return request;
        
    }

}

//NewMove command
public abstract class MoveStatement1 : Statement
{
    public Vector3 target;
    public int movementValueForCanvas = -1;
    public RosSharp.RosBridgeClient.SetPTPCmdServiceClient serviceClient;



    public RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest SendCommandToRos1()
    {
        byte ptpMode_byte;
        byte.TryParse("" + movementValueForCanvas, out ptpMode_byte);
        RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest request =
            new RosSharp.RosBridgeClient.MessageTypes.Dobot.
            SetPTPCmdRequest(ptpMode_byte, target.x, target.y,
            target.z, 0, true);
        return request;
        
    }

}

//BruteMove command definition
public abstract class BruteMove : Statement
{
    public Vector3 target;
    public int movementValueForCanvas = -1;
    public RosSharp.RosBridgeClient.SetPTPCmdServiceClient serviceClient;

    public RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest SendCommandToRos1()
    {
        byte ptpMode_byte;
        byte.TryParse("" + movementValueForCanvas, out ptpMode_byte);
        RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest request =
            new RosSharp.RosBridgeClient.MessageTypes.Dobot.
            SetPTPCmdRequest(ptpMode_byte, target.x, target.y,
            target.z, 0, true);
        return request; 
    }

}
//ClawUp command definition
public class ClawUp : Statement
{
    public bool isClawUp = false;
}


public abstract class MoveToStatement: MoveStatement
{

}
public class MoveToL : MoveToStatement
{
    public MoveToL()
    {
        this.movementValueForCanvas = 0;
    }
}

public class MoveToJ : MoveToStatement
{
    public MoveToJ()
    {
        this.movementValueForCanvas = 1;
    }
}
public class JumpTo : MoveToStatement
{
    public JumpTo()
    {
        this.movementValueForCanvas = 2;
    }
}
public abstract class MoveArcStatement : MoveToStatement
{
    public Vector3 arcPoint;
}
public class MoveArc: MoveArcStatement
{
    public MoveArc()
    {
        this.movementValueForCanvas = 3;
    }
}

public abstract class MoveToStatement1: MoveStatement1
{

}
public class MoveToL1 : MoveToStatement1
{
    public MoveToL1()
    {
        this.movementValueForCanvas = 0;
    }
}

public class MoveToJ1 : MoveToStatement1
{
    public MoveToJ1()
    {
        this.movementValueForCanvas = 1;
    }
}
public class JumpTo1 : MoveToStatement1
{
    public JumpTo1()
    {
        this.movementValueForCanvas = 2;
    }
}
public abstract class MoveArcStatement1 : MoveToStatement1
{
    public Vector3 arcPoint;
}
public class MoveArc1: MoveArcStatement1
{
    public MoveArc1()
    {
        this.movementValueForCanvas = 3;
    }
}

//types defined for Brute Move command
public abstract class MoveBrute: BruteMove
{

}
public class BruteMoveToL1 : MoveBrute
{
    public BruteMoveToL1()
    {
        this.movementValueForCanvas = 0;
    }
}

public class BruteMoveToJ1 : MoveBrute
{
    public BruteMoveToJ1()
    {
        this.movementValueForCanvas = 1;
    }
}
public class BruteJumpTo1 : MoveBrute
{
    public BruteJumpTo1()
    {
        this.movementValueForCanvas = 2;
    }
}
public abstract class BruteMoveArcStatement1 : MoveBrute
{
    public Vector3 arcPoint;
}
public class BruteMoveArc1: MoveBrute
{
    public BruteMoveArc1()
    {
        this.movementValueForCanvas = 3;
    }
}



/// <summary>
/// For Lejos NXT movestatement --> Class Definition - Begin
/// </summary>

public class LejosMoveStatement : Statement
{
    public string moveToX = "Move to X";
    public int x;
    public int z;
    public string moveToZ = "Move to Z";

    public LejosMoveStatement()
    {

    }

}
/// <summary>
/// For Lejos NXT movestatement --> Class-Definition - End
/// </summary>
