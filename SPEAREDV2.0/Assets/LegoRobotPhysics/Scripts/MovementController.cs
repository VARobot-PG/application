using System.Collections.Generic;
using UnityEngine;

/* The Movement controller is the brain of the robot simulation. It keeps track of all subsystems and combines their information 
 * to make the robot model act appropriately. */
public class MovementController : MonoBehaviour
{
    public TorqueAdder taLeft;          /* The Torque Adder for the left wheel */
    public TorqueAdder taRight;         /* The Torque Adder for the right wheel */
    public UltrasonicRaycast raycast;   /* The raycaster from the ultrasonic sensor */
    public ClawControl claw;            /* The control class on the claw */
    public ClickerController clicker;   /* The control class on the claw */

    public int goTorque = 2100;          /* Strength (Torque) with which the wheels are turned, must be adjusted on size changes due to different weight */
    public int speed = 720;          /* Speed with which the wheels should turn, in degrees per second */
    public int directionTolerance = 5;  /* How much off the ideal Vector from the robot to the goal can the robot go before we correct it (in degrees)*/
    public float positionTolerance = 1; /*How much off the goal position can the robot be to still count as at goal*/
    public int rotationTolerance = 15;  /* How much off the ideal rotation from the robot to the goal can the robot go before we correct it (in degrees)
                                          NOTE: robot currently turns an extra about 12 degrees after stopping, so tolerance of 15 works quite well*/
    public bool enableLoggingMessages = false;  /* Turn off to avoid logging messages spamming the console */
    public bool autostartTestrun = false;       /* For debugging: automatically starts some test movements to see if everything works */
    public float avoidanceDistance = 10;           /* The distane the robot drives to avoid an obstacle */
    public float robotHeight = 0;

    public enum states { RIGHTTURN, LEFTTURN, MOVE, BRUTEMOVE, MOVEAVOID, GRAB, RELEASE, STOP }; /* States the robot can be in */
    public states state;                /* Current state of robot */
    private Vector3 goalPosition;       /* The position that we want to go to in the current state */
    private Vector3 originRotation;     /* The rotation when entering the current state */
    private Vector3 goalRotation;       /* The rotation we want to achieve in the current state */
    private Vector3 goalPositionTmp;    /* Variable to save the goal position if we have to override it for an obstacle avoidance maneuvre */
    private bool turnToAvoidFlag = false;/* If the robot is turning to avoid an obstacle */

    private List<states> statesList = new List<states> { };         /* The states the robot should go through */
    private List<Vector3> positionsList = new List<Vector3> { };    /* The positions the robot should go to in the different states. 
                                                                       If a state does not require a position (e.g. claw movement), put any Vector3 in there */

    private Vector3 testPosition = new Vector3(-15, 5, 25);//new Vector3(3, 0, 3.5f);

    /* On start just set logging of subcomponents and feed in test states if auto testrun is enabled, warn if positiontolerance is too big for avoidancedistance */
    private void Start()
    {
        raycast.setEnableLoggingMessages(enableLoggingMessages);
        claw.setEnableLoggingMessages(enableLoggingMessages);
        taLeft.setEnableLoggingMessages(enableLoggingMessages);
        taRight.setEnableLoggingMessages(enableLoggingMessages);
        clicker.setEnableLoggingMessages(enableLoggingMessages);

        robotHeight = this.transform.position.y;


        if (avoidanceDistance <= positionTolerance+0.1f)
        {
            Debug.LogWarning("The avoidanceDistance is smaller than or very similar to the positionTolerance. This might break the obstacle avoidance.");
        }

        if (autostartTestrun)
        {
            statesList = new List<states> { states.GRAB, states.MOVE, states.RIGHTTURN, states.RELEASE };
            positionsList = new List<Vector3> { testPosition, testPosition, testPosition, testPosition };
            simulateProgram(statesList, positionsList);
        }
    }

    /* Call this (maybe from outside of the simulation) to simulate a robot program. 
     * states: The states the robot should go through
     * goals:  The positions the robot should go to in the different states. 
     * If a state does not require a position (e.g. claw movement), put any Vector3 in there
     */
    public void simulateProgram(List<states> states, List<Vector3> goals)
    {
        Debug.Log("Simulating Program");
        statesList = states;
        positionsList = goals;
        //set all y coordinates to 0 because curently the program inputs them in a horrible way
        for(int i = 0; i<positionsList.Count;i++)
        //foreach(Vector3 v in positionsList)
        {
            Vector3 v = new Vector3();
            v.Set(Mathf.Round(positionsList[i].x*100f)/100f, robotHeight, Mathf.Round(positionsList[i].z * 100f) / 100f);
            positionsList[i] = v;
        }
        Debug.Log("ROBOT Commands:");
        for(int i = 0; i<states.Count; i++)
        {
            Debug.Log("#####"+states[i] + " " + goals[i].ToString("F1"));
        }
        nextState();
    }

    /* Called by update methods when a state is fulfilled, pops the next state and calls setState on it
     */
    private void nextState()
    {
        if (statesList.Count > 0)
        {
            setState(statesList[0], positionsList[0]);
            if (enableLoggingMessages) { Debug.Log("Executing " + statesList[0] + " now"); }
            statesList.RemoveAt(0);
            positionsList.RemoveAt(0);
        }
        else
        {
            if (enableLoggingMessages) { Debug.Log("All commands executed"); }
        }
    }

    /* If setState is ever called only with a state, call the 'proper' setState method with some testPosition*/
    public void setState(states state)
    {
        setState(state, testPosition);
    }

    /* setState sets up the original and goal rotations and positions for the given state 
     * state: The state the robot should go through
     * goal:  The position the robot should go to in the different states. 
     * If a state does not require a position (e.g. claw movement), put any Vector3 in there
     */
    public void setState(states state, Vector3 goal)
    {
        this.state = state;
        originRotation = this.transform.forward;
        switch (state)
        {
            case states.LEFTTURN:
                goalRotation = this.transform.right * -1;
                break;
            case states.RIGHTTURN:
                goalRotation = this.transform.right;
                break;
            case states.MOVE:
                goalPosition = goal;
                if (enableLoggingMessages) { Debug.Log("In MOVE, goal: " + goalPosition.ToString("F1")); }
                break;
            case states.MOVEAVOID:
                //Moveavoid is special, it os not a state to be called from the states list, but is set internally when the robot meets an obstacle to avoid
                if (enableLoggingMessages) { Debug.Log("Setting moveavoid state, goalPosition is " + goalPosition); }
                goalPositionTmp = goalPosition;
                goalPosition = goal;
                if (enableLoggingMessages) { Debug.Log("My Position: " + this.transform.position + " new position: " + goalPosition); }
                break;

        }
        goalPosition = goal;
    }

    /* Makes the robot turn left (one wheel forwards, one backwards)*/
    public void TurnLeft()
    {
        if (enableLoggingMessages) { Debug.Log("Turning Left"); }
        //taLeft.setY(goTorque);
        //taRight.setY(-goTorque);
        taLeft.setMotor(speed / 3, goTorque);
        taRight.setMotor(speed / 3, goTorque);
    }

    /* Makes the robot turn right (one wheel forwards, one backwards)*/
    public void TurnRight()
    {
        if (enableLoggingMessages) { Debug.Log("Turning Right"); }
        //taRight.setY(goTorque);
        //taLeft.setY(-goTorque);
        taLeft.setMotor(-speed / 3, goTorque);
        taRight.setMotor(-speed / 3, goTorque);
    }

    /* Makes the robot go straight forwards (turn both wheels in the same direction) */
    public void GoStraight()
    {
        if (enableLoggingMessages) { Debug.Log("Going forwards"); }
        //taRight.setY(goTorque);
        //taLeft.setY(-goTorque);
        //taLeft.transform.parent.transform.localRotation = taRight.transform.parent.transform.localRotation;
        taLeft.setMotor(-speed, goTorque);
        taRight.setMotor(speed, goTorque);
        taLeft.transform.parent.transform.localRotation = taRight.transform.parent.transform.localRotation;
    }

    /* Stops the robot (stops applying torque force to the wheels) */
    public void Stop()
    {
        if (enableLoggingMessages) { Debug.Log("Stopping"); }
        //taRight.setY(0);
        //taLeft.setY(0);
        taLeft.setMotor(0, goTorque);
        taRight.setMotor(0, goTorque);
    }

    /* Fixed Update is called by Unity regularly. From here, call the specialized update methods depending on the state */
    private void FixedUpdate()
    {
        if (enableLoggingMessages) { Debug.Log("Update: GoalPosition: " + goalPosition + " , tmpgoal: " + goalPositionTmp); }
        switch (state)
        {
            case states.LEFTTURN:
                if (enableLoggingMessages) { Debug.Log("Doing Leftturn, original rotation (vector): " + originRotation + ", GoalRotation: " + goalRotation + ", current rotation (vector): " + this.transform.forward); }
                turnUpdate();
                break;

            case states.RIGHTTURN:
                if (enableLoggingMessages) { Debug.Log("Doing Rightturn, original rotation (vector): " + originRotation + ", GoalRotation: " + goalRotation + ", current rotation (vector): " + this.transform.forward); }
                turnUpdate();
                break;

            case states.BRUTEMOVE:
                if (enableLoggingMessages) { Debug.Log("Brutemove, current position: " + this.transform.position + " Goalposition: " + goalPosition); }
                bruteMoveUpdate();
                break;

            case states.MOVE:
                if (enableLoggingMessages) { Debug.Log("Move, current position: " + this.transform.position + " Goalposition: " + goalPosition); }
                moveUpdate();
                break;

            case states.MOVEAVOID:
                if (enableLoggingMessages) { Debug.Log("Move Avoid, current position: " + this.transform.position + " Goalposition: " + goalPosition); }
                moveAvoidUpdate();
                break;

            case states.GRAB:
                if (enableLoggingMessages) { Debug.Log("Grabbing Ball"); }
                grabUpdate();
                break;

            case states.RELEASE:
                if (enableLoggingMessages) { Debug.Log("Releasing Ball"); }
                releaseUpdate();
                break;
        }
    }

    /* If the robot should turn, turn is as long as it is not tolerably close to its goal rotation */
    private void turnUpdate()
    {
        if (!similarDirection(this.transform.forward, goalRotation))
        {
            if (enableLoggingMessages) { Debug.Log("Direction not similar, turning"); }
            if (Vector3.SignedAngle(this.transform.forward, goalRotation, Vector3.up) < 0)
            {
                TurnLeft();
            }
            else
            {
                TurnRight();
            }
        }
        else
        {
            if (enableLoggingMessages) { Debug.Log("TURN COMPLETE"); }
            Stop();
            nextState();
        }
    }

    /* Brute move does not take care of obstacles, it lets the robot go towards its goal until it is acceptably close to it. 
     * If the robot rotates too far away from the ideal direction towards the goal, it is rotated to correct that and then goes on*/
    public void bruteMoveUpdate()
    {
        if (!similarPosition(this.transform.position, goalPosition))
        {
            if (enableLoggingMessages) { Debug.Log("are not similar, forward:" + this.transform.forward + " goalvector: " + (goalPosition - this.transform.position).normalized); }
            if (similarDirection(this.transform.forward, goalPosition - this.transform.position) || closeAngleTolerance(this.transform.position, goalPosition, this.transform.forward))
            {
                GoStraight();
            }
            else
            {
                if (Vector3.SignedAngle(this.transform.forward, goalPosition - this.transform.position, Vector3.up) < 0)
                {
                    TurnLeft();
                }
                else
                {
                    TurnRight();
                }
            }
        }
        else
        {
            Stop();
            if (!turnToAvoidFlag)
            {
                /*turnToAvoidFlag means the robot is currently avoiding an obstacle, so when the goal is reached, 
                it should go back to the original state (before activating MOVEAVOID), not the next one*/
                nextState();
            }
            if (enableLoggingMessages) { Debug.Log("REACHED GOAL (Yay)"); }
        }
    }

    /* MoveAvoidUpdate calls brutemove as long as the robot is too far from the (avoidance) goal position, when it has arrived, it turns off the turntoavoid flag and returns to normal function */
    private void moveAvoidUpdate()
    {
        if (turnToAvoidFlag)
        {
            if (!similarPosition(this.transform.position, goalPosition))
            {
                bruteMoveUpdate();
            }
            else
            {
                if (enableLoggingMessages) { Debug.Log("Turntoavoid false"); }
                turnToAvoidFlag = false;
                if (enableLoggingMessages) { Debug.Log("MOVEAVOID done, goalPositionTmp = " + goalPositionTmp); }
                setState(states.MOVE, goalPositionTmp);
            }
        }
    }

    /* MoveUpdate moves, but avoids obstacles. If the robot faces an obstacle, if activates MOVEAVOID and sets the turntoavoid flag to aoid the obstacle,
     * otherwise, it just does a brutemove update
     */
    public void moveUpdate()
    {
        if (raycast.facingThing(5f))
        {
            if (enableLoggingMessages) { Debug.Log("Move Update: facing thing, going into movex"); }
            turnToAvoidFlag = true;
            if (Vector3.SignedAngle(this.transform.forward, goalPosition - this.transform.position, Vector3.up) < 0)
            {
                //turn Left and move to avoid
                setState(states.MOVEAVOID, this.transform.position + (this.transform.right * (-1) * avoidanceDistance));
            }
            else
            {
                //turn Left and move to avoid
                setState(states.MOVEAVOID, this.transform.position + (this.transform.right * avoidanceDistance));
            }
        }
        else
        {
            bruteMoveUpdate();
        }
    }

    /* Grab Update initiates the clawdown movement if the claw is not down and calls the next state 
     * if the claw is in down position or the clicker is pressed (and therefore a ball is 'caught' in the claw)
     */
    public void grabUpdate()
    {
        if (claw.getClawPosition() == ClawControl.clawpositions.UP || claw.getClawPosition() == ClawControl.clawpositions.INBETWEEN && claw.getState() != ClawControl.clawstates.MOVEDOWN)
        {
            claw.setState(ClawControl.clawstates.MOVEDOWN);
        }
        else
        {
            if (claw.getClawPosition() == ClawControl.clawpositions.DOWN||clicker.isPressed())
            {
                nextState();
            }
        }
    }

    /* ReleaseUpdate initiates the clawup movement if the claw is not up and calls the next state 
     * if the claw is in up position or the clicker not pressed anymore (and therefore a ball is not the claw anymore)
     */
    public void releaseUpdate()
    {
        if (claw.getClawPosition() == ClawControl.clawpositions.DOWN || claw.getClawPosition() == ClawControl.clawpositions.INBETWEEN && claw.getState() != ClawControl.clawstates.MOVEUP)
        {
            claw.setState(ClawControl.clawstates.MOVEUP);
        }
        else
        {
            if (claw.getClawPosition() == ClawControl.clawpositions.UP || !clicker.isPressed())
            {
                nextState();
            }
        }
    }

    /* With smaller scales, the angle between the robot and the goal can vary more strongly even through minor differences. 
     * Therefore, this method adds tolerance on the general directionTolerance (the one for the driving direction, not the rotation direction) 
     * to keep the robot from trying to adjust the driving direction too often */
    private bool closeAngleTolerance(Vector3 currentPos, Vector3 goalPos, Vector3 currentDir)
    {
        currentPos.y = 0;
        goalPos.y = 0;
        currentDir.y = 0;
        float distance = System.Math.Abs((currentPos - goalPos).magnitude);
        float angle = Vector3.Angle(currentDir, (goalPos - currentPos));
        if (distance < 5f)
        {
            return angle < directionTolerance * 5;
        }
        else
        {
            if (distance < 10f)
            {
                return angle < directionTolerance * 3;
            }
            else
            {
                if (distance < 15f)
                {
                    return angle < directionTolerance * 2;
                }
            }
        }
        return false;
    }

    /* Checks if two vectors are similar in direction, returns if they are less than directionTolerance apart from each other */
    private bool similarDirection(Vector3 v1, Vector3 v2)
    {
        v1.y = 0;
        v2.y = 0;
        if (enableLoggingMessages) { Debug.Log("Angle: " + (Vector3.SignedAngle(v1, v2, Vector3.up))); }
        return Vector3.Angle(v1, v2) < directionTolerance;
    }
    /* Checks if two rotation vectors are similar, returns if they are less than rotationTolerance in difference */
    private bool similarPosition(Vector3 v1, Vector3 v2)
    {
        v1.y = 0;
        v2.y = 0;
        if (enableLoggingMessages) { Debug.Log("Position difference: " + (v1 - v2).magnitude); }
        return System.Math.Abs((v1 - v2).magnitude) < positionTolerance;
    }
}
