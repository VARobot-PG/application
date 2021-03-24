using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotProgramStructure
{
    public Program sampleProgram;
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
        ToggleSuction toggleSuction = new ToggleSuction();
        toggleSuction.isSuctionEnabled = true;
        statements.Add(toggleSuction);
        statements = newMovementCommand(dobotCoordsAboveCube, new MoveToJ(), statements);
        statements = newMovementCommand(dobotCoordsCube, new MoveToJ(), statements);
        
        statements = newMovementCommand(dobotCoordsAboveCube2, new MoveToJ(), statements);
        toggleSuction = new ToggleSuction();
        toggleSuction.isSuctionEnabled = false;
        statements.Add(toggleSuction);
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

}
public class Program
{
    public List<Statement> statements;
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