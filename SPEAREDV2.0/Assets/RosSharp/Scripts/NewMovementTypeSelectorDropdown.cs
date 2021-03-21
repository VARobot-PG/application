using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class NewMovementTypeSelectorDropdown : NewUIMovementTypeSelector
{
    public GameObject movementTypeDropdownPrefab;

    public override GameObject generateMovmentTypeSelectorFromStatement(MoveStatement1 moveStatement)
    {
        GameObject dropdownObject;
        dropdownObject = Instantiate(movementTypeDropdownPrefab);
        dropdownObject.TryGetComponent(out RectTransform rectTransformDropDown);
        dropdownObject.TryGetComponent(out TMP_Dropdown dropdown);
        dropdown.value = moveStatement.movementValueForCanvas;
        dropdown.RefreshShownValue();
        return dropdownObject;
    }

    //Brute Move
    public override GameObject generateBruteMovmentTypeSelectorFromStatement(BruteMove bruteStatement)
    {
        GameObject dropdownObject;
        dropdownObject = Instantiate(movementTypeDropdownPrefab);
        dropdownObject.TryGetComponent(out RectTransform rectTransformDropDown);
        dropdownObject.TryGetComponent(out TMP_Dropdown dropdown);
        dropdown.value = bruteStatement.movementValueForCanvas;
        dropdown.RefreshShownValue();
        return dropdownObject;
    }

    public override MoveStatement1 getMoveStatementFromUI(GameObject movementTypeUI)
    {
        List<TMP_Dropdown> dropdowns;
        MoveStatement1 moveStatement;
        dropdowns = new List<TMP_Dropdown>();
        moveStatement =  new MoveToL1();
        movementTypeUI.GetComponentsInChildren<TMP_Dropdown>(dropdowns);    
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            switch (dropdown.value)
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
                    break;
            }
        }
        return moveStatement;
    }

    //Brute Move
    public override BruteMove getBruteMoveFromUI(GameObject movementTypeUI)
    {
        List<TMP_Dropdown> dropdowns;
        BruteMove bruteStatement;
        dropdowns = new List<TMP_Dropdown>();
        bruteStatement =  new BruteMoveToL1();
        movementTypeUI.GetComponentsInChildren<TMP_Dropdown>(dropdowns);    
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            switch (dropdown.value)
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
                    break;
            }
        }
        return bruteStatement;
    }

    public override string nameIdentifier()
    {
        return movementTypeDropdownPrefab.name;
    }
}