package com.defaultcompany.nxtbluetoothplugin;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import android.widget.Toast;

import com.unity3d.player.UnityPlayer;

import java.io.IOException;
import java.util.ArrayList;


public class NXTManager
{
    private static NXTManager nxtManager;

    public static String scriptLocation = "NXTManager";
    public static String deviceName = "NXT";

    public int finalRobotPositionX = 0;
    public int finalRobotPositionY = 0;

    private Context context;
    private Activity activity;
    private UnityPlayer unityPlayer;

    private final int MOVE_UP = 1;
    private final int MOVE_DOWN = 2;
    private final int MOVE_LEFT = 3;
    private final int MOVE_RIGHT = 4;


    private final int INPUT_COMMAND_MOVE = 10000;
    private final int INPUT_COMMAND_CLAW_UP = 20000;
    private final int INPUT_COMMAND_CLAW_DOWN = 30000;

    // Bluetooth properties

    BluetoothAdapter bluetoothAdapter;
    public static String connectedDeviceAddress;
    public static BluetoothDevice connectedBluetoothDevice;

    private static boolean autoEnableBT = false;

    public Brick nxt;

    // region Constructor Methods

    public NXTManager()
    {
        nxtManager = this;
    }

    public static NXTManager nxtManager()
    {
        if(nxtManager == null) {
            nxtManager = new NXTManager();
        }

        return nxtManager;
    }

    // endregion Constructor Methods

    // region Multi-Plugin Methods

    ///////////////////////////////

    public void setContext(Context context)
    {
        this.context = context;
        activity = (Activity)context;
    }

    public void setUnityPlayer(UnityPlayer player)
    {
        this.unityPlayer = player;
    }

    public void setActivity(Activity activity)
    {
        this.activity = activity;
    }

    ///////////////////////////////

    // endregion Multi-Plugin Methods


    public void initialisePlugin()
    {
        Log.d("Unity Plugin: ", "We begin");
        UnityPlayer.UnitySendMessage(scriptLocation, "throwAlertUpdate", "Initialising NXT Plugin!");

        // Perform initial checks for bluetooth

        performInitialChecks();

        // Connect to the defined device

        performDeviceConnection();
    }

    public void performInitialChecks()
    {
        // Simple Bluetooth test

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If we have no bluetooth the return

        if (bluetoothAdapter == null)
        {
            performCleanup();
            showToast("This application requires a bluetooth capable device");
            return;
        }

        // showToast("Bluetooth Adapter Found!");

        if (!bluetoothAdapter.isEnabled())
        {
            Log.d("Unity: ", "Bluetooth is off");
            UnityPlayer.UnitySendMessage(scriptLocation, "throwBluetoothAlert", "false");

            if(autoEnableBT)
            {
                requestBluetoothEnabled();

                while (!bluetoothAdapter.isEnabled())
                {
                    // Perform timeout here!
                }
            }
            else
            {
                performCleanup();
            }
        }
    }

    public void  performDeviceConnection()
    {
        if(!bluetoothAdapter.isEnabled())
        {
            showToast("Please enable bluetooth and try again!");
            return;
        }
        else
        {
            UnityPlayer.UnitySendMessage(scriptLocation, "throwBluetoothAlert", "true");
        }

        // All checks are passed so lets begin the connection!

        showToast("Initialising bluetooth connection!");

        // Get the bluetooth devices which are paired
        // with the system.

        nxt = new Brick(this, deviceName);
        nxt.connect();
    }


    ArrayList<Float> commandsToNavigateNxtRobot = new ArrayList();


    public void navigateNXTRobot() throws IOException {
        //TODO: Move the robot here --> use arraylist

        Log.d("UNITY NXT: ", "final size: " + commandsToNavigateNxtRobot.size());
        for(float value: commandsToNavigateNxtRobot){
            Log.d("UNITY NXT: ", "Value in the list: "+ value);
            BluetoothConnection.dataOutputStream.writeFloat(value);
            BluetoothConnection.dataOutputStream.flush();
        }

        commandsToNavigateNxtRobot.clear();
        if(BluetoothConnection.dataOutputStream == null){
            Log.d("NXT Bluetooth: ", "NO OUTPUT STREAM");
        }
    }

    public void resetNXTPos(){
        commandsToNavigateNxtRobot.add((float) CommandType.RESET.ordinal());
    }

    public void sendClawCommandToNxt(String commandType){
        if(commandType.equals("CLAW_UP")){
            commandsToNavigateNxtRobot.add((float) CommandType.CLAW_UP.ordinal());
        }
        else if(commandType.equals("CLAW_DOWN")){
            commandsToNavigateNxtRobot.add((float) CommandType.CLAW_DOWN.ordinal());
        }
    }

    public void moveNXTBack(float x, float z){
        commandsToNavigateNxtRobot.add((float) CommandType.BACK.ordinal());

        Log.d("NXT Bluetooth: " , "BACK");
        Log.d("NXT Bluetooth", "x: " + x );
        Log.d("NXT Bluetooth", "z: " + z);

        commandsToNavigateNxtRobot.add(x);
        commandsToNavigateNxtRobot.add(z);
    }

    // ADDITIONAL CHANGES - BEGIN //
    private float[][] translationAndRotationMatrixCreator(float x, float y, float angleOfRobot){

        float[][] translationRotationMatrix = {
                {(float) Math.cos(angleOfRobot), -((float) Math.sin((float) angleOfRobot)) , 0},
                {(float) Math.sin(angleOfRobot), ((float) Math.cos((float) angleOfRobot)) , 0},
                {x , y , 1}
        };

        return translationRotationMatrix;
    }
    // ADDITIONAL CHANGES - END//


    public void sendCoordsToNXT(float x , float z){

        try {
            commandsToNavigateNxtRobot.add((float) CommandType.MOVE.ordinal());

            Log.d("NXT Bluetooth: " , "MOVE");
            Log.d("NXT Bluetooth", "x: " + x );
            Log.d("NXT Bluetooth", "z: " + z);

            commandsToNavigateNxtRobot.add(x);
            commandsToNavigateNxtRobot.add(z);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }


    public void performCleanup()
    {

    }

    public void requestBluetoothEnabled()
    {
        Intent turnOn = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        activity.startActivityForResult(turnOn, 0);

        UnityPlayer.UnitySendMessage(scriptLocation, "throwBluetoothAlert", "true");
    }

    public void sendMessage(String event, String message)
    {
        UnityPlayer.UnitySendMessage(scriptLocation, event, message);
    }

    public void showToast(final String message)
    {
        try
        {
            activity.runOnUiThread(new Runnable() {
                public void run() {
                    Toast.makeText(activity, message, Toast.LENGTH_SHORT).show();
                }
            });
        }
        catch (Exception ex)
        {
            UnityPlayer.UnitySendMessage(scriptLocation, "throwAlertUpdate", "Error: " + ex.getMessage());
        }
    }

}
