using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class MovementTypeSelectorDropdown : IUIMovementTypeSelector
{
    public GameObject movementTypeDropdownPrefab;

    public override GameObject generateMovmentTypeSelectorFromStatement(MoveStatement moveStatement)
    {
        GameObject dropdownObject;
        dropdownObject = Instantiate(movementTypeDropdownPrefab);
        dropdownObject.TryGetComponent(out RectTransform rectTransformDropDown);
        dropdownObject.TryGetComponent(out TMP_Dropdown dropdown);
        dropdown.value = moveStatement.movementValueForCanvas;
        dropdown.RefreshShownValue();
        return dropdownObject;
    }

    public override MoveStatement getMoveStatementFromUI(GameObject movementTypeUI)
    {
        List<TMP_Dropdown> dropdowns;
        MoveStatement moveStatement;
        dropdowns = new List<TMP_Dropdown>();
        moveStatement =  new MoveToL();
        movementTypeUI.GetComponentsInChildren<TMP_Dropdown>(dropdowns);    
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            switch (dropdown.value)
            {
                case 0:
                    moveStatement = new MoveToL();
                    break;
                case 1:
                    moveStatement = new MoveToJ();
                    break;
                case 2:
                    moveStatement = new JumpTo();
                    break;
                case 3:
                    moveStatement = new MoveArc();
                    break;
                default:
                    break;
            }
        }
        return moveStatement;
    }

    public override string nameIdentifier()
    {
        return movementTypeDropdownPrefab.name;
    }
}