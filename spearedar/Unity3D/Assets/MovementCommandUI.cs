using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class MovementCommandUI : ICommandUIRepresentation
{
    public GameObject statementButtonAdditonsPrefab;
    public GameObject suctionTogglePrefab;
    public IUICoordinateSelector coordinateSelector;
    public IUIMovementTypeSelector movementTypeSelector;
    private Rect currentRect = new Rect(0, 0, 0, 0);

    public override void changeCommandTypeComponent(IUIMovementTypeSelector uImovementTypeSelector)
    {
        this.movementTypeSelector = uImovementTypeSelector;
    }

    public override void changeParameterComponent(IUICoordinateSelector uICoordinateSelector)
    {
        this.coordinateSelector = uICoordinateSelector;
    }

    public override GameObject constructCommandUIComponent(string commandUIName, Statement statement)
    {
        GameObject commandObject;
        GameObject dropdownObject;
        GameObject statementButtonAdditons;
        MoveStatement moveStatement;
        GameObject coordinateDisplay;
        commandObject = new GameObject();
        commandObject.name = commandUIName;
        if (statement.GetType().IsSubclassOf(typeof(MoveStatement))) {
            moveStatement = (MoveStatement)statement;
            dropdownObject = movementTypeSelector.generateMovmentTypeSelectorFromStatement(moveStatement);
            dropdownObject.transform.SetParent(commandObject.transform, false);
            coordinateDisplay = coordinateSelector.createCoordinatesSelector(moveStatement);
            coordinateDisplay.transform.SetParent(commandObject.transform, false);
        }
        else
        {
            if(statement is ToggleSuction)
            {
                GameObject toggleSuctionObject = Instantiate(suctionTogglePrefab);
                ToggleSuction statementToggle = (ToggleSuction)statement;
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
            }
        }
        statementButtonAdditons = Instantiate(statementButtonAdditonsPrefab);
        statementButtonAdditons.transform.SetParent(commandObject.transform, false);
        //statementButtonAdditons.SetActive(false);
               
        return commandObject;
    }

        public override Statement generateStatementFromUI(GameObject commandContainer)
        {
            bool isMoveStatement = true;
        MoveStatement moveStatement = new MoveToJ();
        foreach (Transform child in commandContainer.transform)
            {
                if (child.name.StartsWith("ToggleSuction"))
                {
                ToggleSuction toggleSuctionCommand = new ToggleSuction();
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
            }
        if (isMoveStatement)
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
            
        }
        return moveStatement;



    }

    public override Rect getCommandObjectBoundaries()
    {
        return this.currentRect;
    }


}
