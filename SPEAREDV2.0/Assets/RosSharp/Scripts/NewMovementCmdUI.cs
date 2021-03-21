using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class NewMovementCmdUI : NewICmdRepresentation
{
    public GameObject statementButtonAdditonsPrefab;
    //public GameObject brutePrefab;
    public GameObject brutePrefab;
    //public GameObject ClawUp;
    public GameObject clawPrefab;
    public NewUICoordinateSelector coordinateSelector;
    public NewUIMovementTypeSelector movementTypeSelector;
    private Rect currentRect = new Rect(0, 0, 0, 0);
    //flags to identify commands
    bool moveType, bruteType; //default
    int countmoveType = 0;
    int countbruteType = 0;

    public override void changeCommandTypeComponent(NewUIMovementTypeSelector uImovementTypeSelector)
    {
        this.movementTypeSelector = uImovementTypeSelector;
    }

    public override void changeParameterComponent(NewUICoordinateSelector uICoordinateSelector)
    {
        this.coordinateSelector = uICoordinateSelector;
    }

    public override GameObject constructCommandUIComponent(string commandUIName, Statement statement)
    {   //In this method we have created a flag to identify which type of statement will be handled by generateStatementFromUI()
        GameObject commandObject;
        GameObject dropdownObject;
        GameObject statementButtonAdditons;
        MoveStatement1 moveStatement;
        //Brute Move
        BruteMove bruteStatement;
        GameObject coordinateDisplay;
        commandObject = new GameObject();
        commandObject.name = commandUIName;
        //Statement type in constructCommandUIComponent() is decided here and is rendered in Touch Input
        if (statement.GetType().IsSubclassOf(typeof(MoveStatement1))) {
            moveStatement = (MoveStatement1)statement;
            dropdownObject = movementTypeSelector.generateMovmentTypeSelectorFromStatement(moveStatement);
            dropdownObject.transform.SetParent(commandObject.transform, false);
            //denote the statementType flag value
            moveType = true;     
            countmoveType++;
            coordinateDisplay = coordinateSelector.createCoordinatesSelector(moveStatement);
            coordinateDisplay.transform.SetParent(commandObject.transform, false);
        }
        //constructing command UI component for Brute Move
        else if (statement.GetType().IsSubclassOf(typeof(BruteMove))) {
            //BruteMove Prefab
            GameObject brutePrefabObject = Instantiate(brutePrefab);
            brutePrefabObject.transform.SetParent(commandObject.transform, false);
            bruteStatement = (BruteMove)statement;
            dropdownObject = movementTypeSelector.generateBruteMovmentTypeSelectorFromStatement(bruteStatement);
            dropdownObject.transform.SetParent(commandObject.transform, false);
            //denote the statementType flag value
            bruteType = true;
            countbruteType++;
            coordinateDisplay = coordinateSelector.createBruteCoordinatesSelector(bruteStatement);
            //BruteMove coordinates
            coordinateDisplay.transform.SetParent(commandObject.transform, false);
        }
        else
        {
            if (statement is ClawUp)
            {   
                GameObject clawPrefabObject = Instantiate(clawPrefab);
                ClawUp statementToggle = (ClawUp)statement;
                ToggleClaw toggleClawScript = clawPrefabObject.GetComponentInChildren<ToggleClaw>();
                if(toggleClawScript != null)
                {
                    if(toggleClawScript.clawUp == statementToggle.isClawUp)
                    {

                    }
                    else
                    {
                        toggleClawScript.toggleClaw();
                    }
                }
                clawPrefabObject.transform.SetParent(commandObject.transform, false);    
            }    

                /*
                if(statement is ToggleSuction1)
                {   /*
                    GameObject toggleSuctionObject = Instantiate(suctionTogglePrefab);
                    ToggleSuction1 statementToggle = (ToggleSuction1)statement;
                    ToggleSuctionCup toggleSuctionCupScript = toggleSuctionObject.GetComponentInChildren<ToggleSuctionCup>();
                    if(toggleSuctionCupScript != null)
                    {
                        if(toggleSuctionCupScript.suctionOn == statementToggle.isSuctionEnabled)
                        {

                        }
                        else
                        {
                            toggleSuctionCupScript.toggleSuction();
                        }
                    }
                    toggleSuctionObject.transform.SetParent(commandObject.transform, false);    
                }    */
        }
        statementButtonAdditons = Instantiate(statementButtonAdditonsPrefab);
        statementButtonAdditons.transform.SetParent(commandObject.transform, false);
        //statementButtonAdditons.SetActive(false);
               
        return commandObject;
    }

        public override Statement generateStatementFromUI(GameObject commandContainer)
        {
        MoveStatement1 moveStatement = new MoveToJ1();
        BruteMove bruteStatement = new BruteMoveToJ1();
        Statement defaultStatement = new MoveToL1();
        bool isMoveStatement = true;
        //also the below toggle suction is not needed for NXT robot
        /*
        foreach (Transform child in commandContainer.transform)
            {
                if (child.name.StartsWith("ToggleSuction"))
                {
                ToggleSuction1 toggleSuctionCommand = new ToggleSuction1();
                if (child.name.Contains("false"))
                {
                    toggleSuctionCommand.isSuctionEnabled = false;
                }
                if (child.name.Contains("true"))
                {
                    toggleSuctionCommand.isSuctionEnabled = true;
                }
                return toggleSuctionCommand;
                isMoveStatement = false;
                }   
            }  */
        //Deciding on statement type
           /* foreach (Transform child in commandContainer.transform)
            {
                if (child.name.StartsWith("ClawUp"))
                {
                    ClawUp clawupCommand = new ClawUp();
                    if (child.name.Contains("false"))
                    {
                        clawupCommand.isClawUp = false;
                    }
                    if (child.name.Contains("true"))
                    {
                        clawupCommand.isClawUp = true;
                    }
                    return clawupCommand;
                    isMoveStatement = false;
                }
            } */
        
            foreach (Transform child in commandContainer.transform)
            {
                if (child.name.StartsWith("Brute"))
                {
                List<Toggle> toggles = new List<Toggle>();
                commandContainer.GetComponentsInChildren<Toggle>(toggles);

                List<Transform> childTransforms = new List<Transform>();
                commandContainer.GetComponentsInChildren<Transform>(childTransforms);
                foreach (Transform childTransform in childTransforms)
                {

                    if (childTransform.name.Contains(movementTypeSelector.nameIdentifier()))
                    {
                        bruteStatement = movementTypeSelector.getBruteMoveFromUI(childTransform.gameObject);
                    }
                    else if (childTransform.name.Contains(coordinateSelector.nameIdentifier()))
                    {
                        bruteStatement.target = coordinateSelector.getCoordinateValues(childTransform.gameObject);
                    }

                }
                //Returning bruteStatement
                bruteType = false;
                //countbruteType--;
                //Debug 
                Debug.Log("Brute found in generateStatementFromUI");
                isMoveStatement = false;
                return bruteStatement;
                }

                if (child.name.StartsWith("Move"))
                {
                    List<Toggle> toggles = new List<Toggle>();
                    commandContainer.GetComponentsInChildren<Toggle>(toggles);

                    List<Transform> childTransforms = new List<Transform>();
                    commandContainer.GetComponentsInChildren<Transform>(childTransforms);
                    foreach (Transform childTransform in childTransforms)
                    {

                        if (childTransform.name.Contains(movementTypeSelector.nameIdentifier()))
                        {
                            moveStatement = movementTypeSelector.getMoveStatementFromUI(childTransform.gameObject);
                        }
                        else if (childTransform.name.Contains(coordinateSelector.nameIdentifier()))
                        {
                            moveStatement.target = coordinateSelector.getCoordinateValues(childTransform.gameObject);
                        }

                    }
                    //Returning moveStatement
                    Debug.Log("Move found in generateStatementFromUI");
                    return moveStatement;
                }
                //ClawUp Command
                if (child.name.StartsWith("Toggle"))
                {
                    ClawUp clawupCommand = new ClawUp();
                    if (child.name.Contains("false"))
                    {
                        clawupCommand.isClawUp = false;
                    }
                    if (child.name.Contains("true"))
                    {
                        clawupCommand.isClawUp = true;
                    }
                //Returning ClawUp
                return clawupCommand;
                    isMoveStatement = false;
                }
        }

           /* if(isMoveStatement)
            {
            List<Toggle> toggles = new List<Toggle>();
            commandContainer.GetComponentsInChildren<Toggle>(toggles);

            List<Transform> childTransforms = new List<Transform>();
            commandContainer.GetComponentsInChildren<Transform>(childTransforms);
            foreach (Transform childTransform in childTransforms)
            {

                if (childTransform.name.Contains(movementTypeSelector.nameIdentifier()))
                {
                    moveStatement = movementTypeSelector.getMoveStatementFromUI(childTransform.gameObject);
                }
                else if (childTransform.name.Contains(coordinateSelector.nameIdentifier()))
                {
                    moveStatement.target = coordinateSelector.getCoordinateValues(childTransform.gameObject);
                }

            }   
                //Returning moveStatement
                Debug.Log("Move found in generateStatementFromUI");
                return moveStatement;
            } */

        //Returning defaultStatement
        //Debug 
        Debug.Log("Default found in generateStatementFromUI");
        return defaultStatement;
    }

    public override Rect getCommandObjectBoundaries()
    {
        return this.currentRect;
    }


}
