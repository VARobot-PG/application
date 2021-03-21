using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using UnityEngine.UI;

public class ExecuteNewGenProg : MonoBehaviour
{
    public Program currentProgram;
    private int currentPosition = 0;
    //todo explain this semaphor
    private bool semaphor_dobot_is_ready = true;
    public List<int> breakpoints = new List<int>();
    public bool breakpointFinished = true;
    //public String idle_topic = "/Dobot_Loader/idle";
    public RosSharp.RosBridgeClient.PausePhysics PhysicsManager;
    public IdleSubscriber MyIdleSubscriber;
    // public RosSharp.RosBridgeClient.RosConnector connector;
    public TimeGhost GeisterZeitJungeEy;
    public MarkerHandler MarkerManager;
    public SetEndeffectorSuctionCup setEndeffectorSuctionCup;
    private bool BreakPointsEnabled = true;
    private List<GameObject> CollidedObjects = new List<GameObject>();
    private bool StepByStepEnabled = false;
    public CodetoCanvasGenNew codeToCanvasGenerator;
    public SphereDrawer sphereDrawer;
    private IEnumerator executionLoop;
    private IEnumerator executionLoopStepByStep;


    private List<Color> origColors = new List<Color>();


    // Start is called before the first frame update
    void Start()
    {
        //PhysicsManager = GameObject.FindObjectOfType<RosSharp.RosBridgeClient.PausePhysics>();
        //PhysicsManager = connector.gameObject.AddComponent<RosSharp.RosBridgeClient.PausePhysics>();
        //PhysicsManager.SetRosConnector(connector);
        //MyIdleSubscriber = connector.gameObject.AddComponent<IdleSubscriber>();
        //setting the topic we want to subscribe to
        //MyIdleSubscriber.Topic = idle_topic;
        //setting the frequency we are pulling updates 0.5f means every 0.5 seconds
        //MyIdleSubscriber.TimeStep = 0.25f;
        //finding the timeGhost manger and attaching it
        //GeisterZeitJungeEy = GameObject.FindObjectOfType<TimeGhost>();
        //finding the MarkerHandler class in order to 
        //MarkerManager = connector.GetComponent<MarkerHandler>();


        ///###########DEBUGING#################
        //breakpoints.Add(3);
        //breakpoints.Add(8);
        //######################################
        //  StartCoroutine(ExecutionLoop());

    }



    // Update is called once per frame
    void Update()
    {

        string formattedCode = "";
        int currentLine = 0;
        string linemodified;
        int currentPosMinus1 = Math.Min(0, currentPosition - 1);
        codeToCanvasGenerator.programCodeFormatted.GetComponent<Text>().text = codeToCanvasGenerator.programCodeUnformatted.GetComponent<Text>().text;
        string[] k = codeToCanvasGenerator.programCodeFormatted.GetComponent<Text>().text.Split('\n');
        foreach (String line in k)
        {
            if (line != "")
            {
                linemodified = line;
                if (currentLine == Math.Max(0, currentPosition - 1))
                    linemodified = "<b>" + line + "</b>";
                if (formattedCode != "")
                    formattedCode += "\n" + linemodified;
                else
                    formattedCode += linemodified;
                currentLine++;
            }
        }
        codeToCanvasGenerator.programCodeFormatted.GetComponent<Text>().text = formattedCode;
        codeToCanvasGenerator.onScreenDobotCodePanel.GetComponent<Text>().text = formattedCode;
    }
    public void StartProgram()
    {
        sphereDrawer.clearTargets();
        foreach (Statement currentStatement in this.currentProgram.statements)
        {
            if (currentStatement.GetType().IsSubclassOf(typeof(MoveStatement1)))
            {
                MoveStatement1 moveStatement = (MoveStatement1)currentStatement;
                sphereDrawer.drawNewTargetSphereAtROSCoords(moveStatement.target);
            }
        }
        currentPosition = 0;
        PhysicsManager.continuePhysics();


    }
    void StopProgram()
    {
        breakpointFinished = true;
        currentPosition = this.currentProgram.statements.Count - 1;
        PhysicsManager.continuePhysics();
    }

    public IEnumerator ExecutionLoop()
    {
        while (true)
        {

            this.ReceiveIdleState(MyIdleSubscriber.isIdle());
            this.continueProgram(this.currentProgram);
            yield return new WaitForSeconds(1);
            //if (MyIdleSubscriber.idle)
            //{
            //Debug.Log("Dobot is idle");
            //}
            this.ReceiveIdleState(MyIdleSubscriber.isIdle());
            yield return new WaitForSeconds(1);
        }
    }

    public IEnumerator StepByStepExecutionLoop()
    {
        while (true)
        {
            this.ReceiveIdleState(MyIdleSubscriber.isIdle());
            yield return new WaitForSeconds(1);
        }

    }
    public bool EnableStepByStep()
    {
        Debug.Log("enabling stepbystep");
        StopCoroutine(executionLoop);
        //StopAllCoroutines();
        //StartCoroutine(executionLoopStepByStep);
        //from now on the continueProgram method has to be triggered manually
        //to continue the program
        StepByStepEnabled = true;
        return StepByStepEnabled;
    }

    public bool DisableStepByStep()
    {
        //this method enables the normal ExecutionLoop that automatically
        // tries to continue the Execution once the Dobot is ready
        Debug.Log("disabling stepbystep");
        if (executionLoopStepByStep == null)
        {
            executionLoopStepByStep = StepByStepExecutionLoop();
        }
        if (executionLoop == null)
        {
            executionLoop = ExecutionLoop();
        }
        try
        {
          //  StopCoroutine(executionLoopStepByStep);
        }
        catch
        {

        }
        StartCoroutine(executionLoop);
        StepByStepEnabled = false;
        return StepByStepEnabled;
    }
    public bool ToggleExecutionLoop()
    {
        if (StepByStepEnabled)
        {
            return DisableStepByStep();
        }
        else
        {
            return EnableStepByStep();
        }
    }
    public void continueProgram()
    {
        if (StepByStepEnabled)
        {
            this.ReceiveIdleState(MyIdleSubscriber.isIdle());
        }
        continueProgram(this.currentProgram);
    }
    //todo wrap executor loop in something that runs infinetly on the side waiting
    public void continueProgram(Program program)
    {
        if (program != null)
        {
            List<Statement> statements = program.statements;
            //warning below can be ignored, it is exactly what we want
            // this unnecessary assignment is done becuae the variable can not be used as
            // loop iteratir
            string log = $"Executing command {currentPosition:G2} of {statements.Count - 1:G2} now.";
            //            for (currentPosition = currentPosition; (currentPosition < statements.Count) && semaphor_dobot_is_ready && breakpointFinished; currentPosition++)
            if ((currentPosition < statements.Count) && semaphor_dobot_is_ready && breakpointFinished)
            {
                
                Debug.Log(log);
                Statement currentStatement = statements[currentPosition];
                if (BreakPointsEnabled && breakpoints.Contains(currentPosition))
                {
                    breakpointFinished = false;
                    Debug.Log("Handling Breakpoint 1");
                    handleBreakpoint(currentStatement);
                }
                //deleting all old Markers before sending the next statement
                MarkerManager.clearMarkers();
                //actually sending the statement
                // executeStatement(currentStatement);
                currentPosition++;
            }
        }
    }

    /*
    //design this blocking untill the statement is actually completed
    private void executeStatement(Statement currentStatement)
    {
        semaphor_dobot_is_ready = false;
        if (connector == null)
        {
            throw new ArgumentException("The Connector instance has to be set");
        }


        RosSocket rosSocket =
            connector.RosSocket;
        if (currentStatement.GetType().IsSubclassOf(typeof(MoveStatement1)))
        {
            MoveStatement1 moveStatement = (MoveStatement1)currentStatement;
            //sphereDrawer.drawSphereAtROSCoords(moveStatement.target);
            RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest req =
                moveStatement.SendCommandToRos1();

            String x = rosSocket.CallService<RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdRequest,
                  RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdResponse>
                  ("/SetPTPCmds", ResponseHandler, req);
            
        }
        else
        {
            if (currentStatement is ToggleSuction)
            {
                // ToggleSuction1 toggleSuction = (ToggleSuction1)currentStatement;
                
                // setEndeffectorSuctionCup.setEndEffectorSuctionCup(toggleSuction.isSuctionEnabled);
            } 
        }
        semaphor_dobot_is_ready = false;


    }
    */

    public void ResponseHandler(RosSharp.RosBridgeClient.MessageTypes.Dobot.SetPTPCmdResponse R)
    {
        //todo result codes are not reliable for the dobot check qued CMdIndex instead
        //todo check
        if (R.result > 0)
        {
            //todo consider somehow using the response codes

        }
      
    }

    public void ReceiveIdleState(bool idle)
    {
        bool error = false;
        if (error)
        {
            semaphor_dobot_is_ready = false;
            //emulate the behaviour of a breakpoint here and stop
            Debug.Log("Handling Breakpoint");
            handleBreakpoint(currentProgram.statements[currentPosition]);
            //todo consider designing a callback to the user telling him why the execution was halted
            
        }
        else
        {
            //free the virtual dobot ressource by increasing the semaphor
            semaphor_dobot_is_ready = idle;

        }

    }
    public void handleCollision(Collider other, GameObject caller)
    {
        //this method is a callback that gets triggered whenever an unwanted
        // collision is detected
        if (BreakPointsEnabled && breakpointFinished)
        {
            
            handleBreakpoint(null,true);
            //mark collided objects
            Renderer[] renderers = caller.gameObject.GetComponentsInChildren<Renderer>();
            foreach (Renderer r in renderers)
            {
                Material[] materials = r.materials;
                foreach (Material m in materials)
                {
                    Color defaultColor = new Color(m.color.r, m.color.g,
                        m.color.b, m.color.a);
                    origColors.Add(defaultColor);
                    m.color = Color.red;
                }
            }
            //disable collider so we don't get more callbacks for now
            Collider collider = caller.GetComponent<Collider>();
            collider.isTrigger = false;
            
            CollidedObjects.Add(caller);

        }
    }

    private void handleBreakpoint(Statement statement, bool collision = false)
    {
        GeisterZeitJungeEy.showTimeGhosts(25, 1);
        PhysicsManager.pausePhysics();
        breakpointFinished = false;
        
        //todo visualize 
        //todo trigger debug view
        //callback to the ui marking the current line
        //todo generate paths and time ghosts etc here

    }

    // callback when continue button is pressed on UI and breakpoint is over
    // this triggers the ui
    public void continueAfterBreakpoint()
    {
        //basically undo all the changes done through the breakpoint
        if (BreakPointsEnabled)
        {
            //undo changes triggered by a collision
            foreach (GameObject gameObject in CollidedObjects)
            {
                //restore trigger in 2 seconds
                IEnumerator coroutine = enableCollider(gameObject, 2.0f);
                StartCoroutine(coroutine);



                //restore original color
                Renderer[] renderers = gameObject.GetComponentsInChildren<Renderer>();
                int colorCounter = 0;
                foreach (Renderer r in renderers)
                {
                    Material[] materials = r.materials;
                    foreach (Material m in materials)
                    {
                        //todo improve and set the objects back to their real color
                        //clone old color

                        m.color = origColors[colorCounter];
                        colorCounter++;
                    }
                }
            }
            CollidedObjects = new List<GameObject>();
            origColors = new List<Color>();

            GeisterZeitJungeEy.hideAllGhosts();
            PhysicsManager.continuePhysics();
            breakpointFinished = true;
        }
       
    }

    private IEnumerator enableCollider(GameObject collidedObject, float wait = 2f)
    {
        yield return new WaitForSeconds(wait);
        Collider collider;
        try
        {
            collider = collidedObject.GetComponent<Collider>();
        }
        catch
        {
            collider = null;
        }
        if (collider != null)
        {
            collider.isTrigger = true;
        }
    }


    //enables or disables all breakpints at once and returns the new state
    // of the switch
    public bool ToggleAllBreakPoints()
    {
        if (BreakPointsEnabled)
        {
            //check if we are currently in a Breakpoint
            // and if yes we'll continue like the user pressed the continue
            //button
            if (!breakpointFinished)
            {
                this.continueAfterBreakpoint();
            }
          
        }
        this.BreakPointsEnabled = !BreakPointsEnabled;
        return this.BreakPointsEnabled;
    }
}
