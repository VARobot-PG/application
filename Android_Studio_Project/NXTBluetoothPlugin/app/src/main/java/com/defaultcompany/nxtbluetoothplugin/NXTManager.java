package com.defaultcompany.nxtbluetoothplugin;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.support.annotation.LongDef;
import android.util.Log;
import android.widget.Toast;

import com.unity3d.player.UnityPlayer;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import javax.bluetooth.RemoteDevice;

import lejos.nxt.Motor;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.pc.comm.NXTCommLogListener;
import lejos.pc.comm.NXTConnector;
import lejos.pc.tools.NXTNotFoundException;
import lejos.pc.tools.Upload;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.ArcRotateMoveController;
import lejos.robotics.navigation.DifferentialPilot;

import lejos.robotics.navigation.Pose;
import lejos.util.Delay;
import lejos.util.PilotProps;


public class NXTManager
{
    private static NXTManager nxtManager;

    public static String scriptLocation = "NXTManager";
    public static String deviceName = "NXT";

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
    ArrayList<Float> commandsToNavigateNxtRobotCheck = new ArrayList();


    public void navigateNXTRobot() throws IOException {
        //TODO: Move the robot here --> use arraylist


//        commandsToNavigateNxtRobotCheck.add((float) CommandType.MOVE.ordinal());
//        commandsToNavigateNxtRobotCheck.add(20.00f);
//        commandsToNavigateNxtRobotCheck.add(0.00f);
//        commandsToNavigateNxtRobotCheck.add((float) CommandType.CLAW_UP.ordinal());
//
//        commandsToNavigateNxtRobotCheck.add((float) CommandType.MOVE.ordinal());
//        commandsToNavigateNxtRobotCheck.add(20.00f);
//        commandsToNavigateNxtRobotCheck.add(-20.00f);
//
//        commandsToNavigateNxtRobotCheck.add((float) CommandType.CLAW_DOWN.ordinal());
//
//        commandsToNavigateNxtRobotCheck.add((float) CommandType.END.ordinal());

        for(float value: commandsToNavigateNxtRobot){
            BluetoothConnection.dataOutputStream.writeFloat(value);
            BluetoothConnection.dataOutputStream.flush();
        }

        if(BluetoothConnection.dataOutputStream == null){
            Log.d("NXT Bluetooth: ", "NO OUTPUT STREAM");
        }
    }

    public void sendClawCommandToNxt(String commandType){
        if(commandType.equals("CLAW_UP")){
            commandsToNavigateNxtRobot.add((float) CommandType.CLAW_UP.ordinal());
        }
        else if(commandType.equals("CLAW_DOWN")){
            commandsToNavigateNxtRobot.add((float) CommandType.CLAW_DOWN.ordinal());
        }
    }

    public void sendCoordsToNXT(float x , float z){

        try {
            //BluetoothConnection.dataOutputStream.writeInt(INPUT_COMMAND_MOVE);
            commandsToNavigateNxtRobot.add((float) CommandType.MOVE.ordinal());
            //BluetoothConnection.dataOutputStream.flush();

            Log.d("NXT Bluetooth: " , "MOVE");
            Log.d("NXT Bluetooth", "x: " + x );
            Log.d("NXT Bluetooth", "z: " + z);

            commandsToNavigateNxtRobot.add(x);
            commandsToNavigateNxtRobot.add(z);
//            BluetoothConnection.dataOutputStream.writeFloat(x);
//            BluetoothConnection.dataOutputStream.flush();
//            BluetoothConnection.dataOutputStream.writeFloat(z);
//            BluetoothConnection.dataOutputStream.flush();
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
//
//    public void sendCodeToNXT(String filePath){
//        Log.d("Unity Path Send Code: ",filePath);
//
//        File folder = new File(filePath);
//
//        if(!folder.exists()){
//            Log.d("Unity: ","Folder being created.");
//            folder.mkdir();
//        }
//
//        Log.d("Unity: ","Folder exists.");
//        String myFile = filePath + "/HalloWelt.nxj";
//        String all = "";
//
//        Log.d("Unity: ", myFile);
//
//        if(!new File(myFile).exists())
//            Log.d("Unity: ", "File not found.");
//
//        try{
//            BufferedReader bufferedReader = new BufferedReader(new FileReader(myFile));
//            String strLine="";
//
//            while((strLine = bufferedReader.readLine()) != null){
//                all = all + strLine;
//            }
//            Log.d("Unity ALL:" , all );
//
//            File file = new File(myFile);
//
//            ////////////////////////////////////////////////////////////
//            NXTConnector nxtConnector = new NXTConnector();
//
//            nxtConnector.addLogListener(new NXTCommLogListener() {
//                @Override
//                public void logEvent(String s) {
//                    Log.d("Unity: ", "Bluetooth Send Log.listener: " + s);
//                }
//
//                @Override
//                public void logEvent(Throwable throwable) {
//                    Log.d("Unity: ", "Bluetooth Send Stack Trace: ");
//                    throwable.printStackTrace();
//                }
//            });
//
//            boolean connected = nxtConnector.connectTo("btspp://");
//
//            if(!connected){
//                Log.d("Unity: ", "NXTConnector: Failed to connect to any NXT");
//            }
//            else {
//                Log.d("Unity: ", "NXTConnector: Connected to NXT");
//            }
//
//
//            //NOT SO GREAT FROM HERE -- YET TO BE CHECKED
//            RemoteDevice nxtRobot = Bluetooth.getKnownDevice(deviceName);
//
//            if(nxtRobot == null){
//                Log.d("Unity: ", " Remote Device: No such NXT Device");
//            }
//            BTConnection btConnection = Bluetooth.connect(nxtRobot);
//
//            if(!(btConnection == null)){
//                Log.d("Unity: ", "RemoteDevice: Connected to NXT");
//            }
//            else{
//                Log.d("Unity: ", "RemoteDevice: NOT Connected to NXT");
//            }
//
//            /////////////////////////////////////////////////////////
//            Upload upload = new Upload();
//
//            String deviceAddress = "";
//            Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
//
//             for(BluetoothDevice device: pairedDevices){
//               if(device.getName().equals(deviceName)){
//                   deviceAddress = device.getAddress();
//                   break;
//               }
//             }
//
//             upload.upload(deviceName, deviceAddress, 2, file, "Hallo Welt", true);
//
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//            Log.d("Unity: ", "File in internal storage not found.");
//        } catch (IOException e) {
//            e.printStackTrace();
//            Log.d("Unity: ", "File in internal storage can not be read.");
//        } catch (NXTNotFoundException e) {
//            Log.d("Unity: ", e.getMessage());
//            Log.d("Unity:" , "Upload to NXT Failed.");
//        }
//    }
}
