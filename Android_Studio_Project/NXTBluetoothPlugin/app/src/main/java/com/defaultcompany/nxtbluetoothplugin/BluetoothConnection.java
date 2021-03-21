package com.defaultcompany.nxtbluetoothplugin;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.AsyncTask;
import android.util.Log;

import com.unity3d.player.UnityPlayer;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.Set;

import lejos.pc.comm.NXTCommFactory;
import lejos.pc.comm.NXTConnector;


public class BluetoothConnection extends Communication implements IConnected
{
    public BluetoothAdapter bluetoothAdapter;
    public Set<BluetoothDevice> pairedDevices;

    public static DataInputStream dataInputStream;
    public static DataOutputStream dataOutputStream;

    public static String scriptLocation = "NXTManager";
    public static BluetoothSocket bluetoothSocket;
    public BluetoothDevice bluetoothDevice;

    private String deviceName;

    public BluetoothClient client;

    NXTConnector nxtConnector;


    private Brick brick;

    public BluetoothConnection(String deviceName, Brick brick)
    {
        this.deviceName = deviceName;
        this.brick = brick;

        System.out.println("Initialising Bluetooth Communication!");
    }

    // region Communication

    @Override
    void open()
    {
        try
        {
            Log.d("Unity: ", "In bluetoothconnection.open");
            new openConnection().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, this);
        }
        catch (Exception ex)
        {
            System.err.println("Failed to open bluetooth connection!");
        }
    }

    @Override
    void close()
    {
        if (bluetoothSocket != null && bluetoothSocket.isConnected())
        {
            try
            {
                bluetoothSocket.close();

                System.out.println("Socket closed: " + bluetoothSocket.isConnected());
                System.out.println("Wait complete end of connection!");

                try
                {
                    Thread.sleep(4000);
                }
                catch (InterruptedException ex)
                {
                    System.err.println("Interrupted during disconnection : " + ex.getMessage());
                }
            }
            catch (Exception e)
            {
                System.err.println("Error during disconnection : " + e.getMessage());
            }
        }
    }

    @Override
    public void connected(BluetoothSocket socket)
    {
        // We have connected so initialise the client!

        client = new BluetoothClient(brick, socket);

        // Set the client going and begin listening for data

        client.execute();

        // Inform the brick that we are setup and ready to go!

        brick.bluetoothConnected();
    }

    @Override
    public void failed()
    {
        // We have a timeout or null response so fail the connection!

        bluetoothSocket = null;
    }

    // endregion Communication

    // region BluetoothConnection Task

    public class openConnection extends AsyncTask<IConnected, String, BluetoothSocket>
    {
        IConnected callback;

        @Override
        protected BluetoothSocket doInBackground(IConnected... params)
        {
            callback = params[0];


            Log.d("Unity: ", "In bluetoothconnection.openconnection");

            System.out.println("Opening Bluetooth Connection!");

            // Simple Bluetooth test

            bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

            // If we have no bluetooth the return false

            if (bluetoothAdapter == null)
                return null;

            // Check for paired devices

            System.out.println("Checking for paired devices!");

            pairedDevices = bluetoothAdapter.getBondedDevices();

            // If we have no paired devices then return false

            if (pairedDevices.size() <= 0) {
                System.out.println("No paired devices exist");
                return null;
            }
            // Check to see if the named device exists

            System.out.println("Checking to see if the named device exists!");

            for (BluetoothDevice device : pairedDevices)
            {
                if(device.getName().equals(deviceName))
                {
                    bluetoothDevice = device;
                }
            }

            // If the device has not been found then return false

            if(bluetoothDevice == null)
            {
                System.out.println("Bluetooth device is null: 160");
                return null;
            }

            System.out.println("The named device exists!");

            // Initialise the connection
            // Set the socket etc here!

            String uuid = "8be7dbdd-ba8d-4efb-82e3-16ad0327a3ab";

            BluetoothSocket tmpSocket;

            try
            {
                // Use the Serial Port Profile
              //  tmpSocket = bluetoothDevice.createRfcommSocketToServiceRecord(java.util.UUID.fromString(uuid));
                tmpSocket =(BluetoothSocket) bluetoothDevice.getClass().getMethod("createRfcommSocket", new Class[] {int.class}).invoke(bluetoothDevice,1);
                bluetoothSocket = tmpSocket;
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } catch (InvocationTargetException e) {
                e.printStackTrace();
            } catch (NoSuchMethodException e) {
                e.printStackTrace();
            }


//            try
//            {
//                bluetoothSocket.connect();
//            }
//            catch (Exception ex)
//            {
//                UnityPlayer.UnitySendMessage(scriptLocation, "throwAlertUpdate", "Not Connected to NXT");
//                System.err.println("Failed to connect to socket!");
//                return null;
//            }


            // using the program already uploaded on the NXT

            nxtConnector = new NXTConnector();
            //TODO: change ddI23 to device name
            boolean connected = nxtConnector.connectTo(deviceName,"", NXTCommFactory.BLUETOOTH);

            if(!connected){
                Log.d("NXT Bluetooth", " NOT CONNECTED");
                UnityPlayer.UnitySendMessage(scriptLocation, "throwAlertUpdate", "Not Connected to NXT");
                return null;
            }

            UnityPlayer.UnitySendMessage(scriptLocation, "throwAlertUpdate", "Connected to NXT");

            dataInputStream = new DataInputStream(nxtConnector.getInputStream());
            dataOutputStream = new DataOutputStream(nxtConnector.getOutputStream());

            System.out.println("Connected to device: " + deviceName);
            NXTManager.connectedDeviceAddress = bluetoothDevice.getAddress();
            NXTManager.connectedBluetoothDevice = bluetoothDevice;

            return bluetoothSocket;
        }

        @Override
        protected void onPostExecute(BluetoothSocket socket)
        {
            if(socket == null)
            {
                callback.failed();
            }
            else
            {
                callback.connected(socket);
            }
        }
    }

    // endregion BluetoothConnection Task

    @Override
    public void finalize() throws Throwable
    {
        super.finalize();
        close();
    }
}
