using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ModifyPTPCmd : MonoBehaviour
{
    public SetPTPCmdServiceClient setPTPCmdService;

    public void onValueChanged(int selection)
    {
        switch (selection)
        {
            case 0:
                setPTPCmdService.ptpMode = 0;
                setPTPCmdService.x = 0;
                setPTPCmdService.y = 200;
                setPTPCmdService.z = 0;
                setPTPCmdService.r = 0;
                setPTPCmdService.isQueued = false;
                break;
            case 1:
                setPTPCmdService.ptpMode = 1;
                setPTPCmdService.x = 200;
                setPTPCmdService.y = 200;
                setPTPCmdService.z = 200;
                setPTPCmdService.r = 0;
                setPTPCmdService.isQueued = false;
                break;
            case 2:
                setPTPCmdService.ptpMode = 0;
                setPTPCmdService.x = 200;
                setPTPCmdService.y = 200;
                setPTPCmdService.z = 200;
                setPTPCmdService.r = 0;
                setPTPCmdService.isQueued = false;
                break;
            default:
                setPTPCmdService.ptpMode = 1;
                setPTPCmdService.x = 0;
                setPTPCmdService.y = 150;
                setPTPCmdService.z = 0;
                setPTPCmdService.r = 0;
                setPTPCmdService.isQueued = false;
                break;
        }
    }
    public void onChangedX(int x)
    {
        setPTPCmdService.x = x;
    }
    public void onChangedY(int y)
    {
        setPTPCmdService.y = y;
    }
    public void onChangedZ(int z)
    {
        setPTPCmdService.z = z;
    }
    public void onChangedPTPMode(int ptpMode)
    {
        setPTPCmdService.ptpMode = ptpMode;
    }

}
