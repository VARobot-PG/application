package com.defaultcompany.nxtbluetoothplugin;

import android.os.AsyncTask;
import android.util.Log;


public class Brick extends AsyncTask<Void,Void,Void>
{
    private NXTManager nxtManager;

    private Communication comm = null;

    public String deviceName;

    public boolean isConnected = false;

    public Brick(NXTManager nxtManager, String deviceName)
    {
        this.nxtManager = nxtManager;
        this.deviceName = deviceName;

        comm = new BluetoothConnection(deviceName, this);
    }

    public void connect()
    {
        Log.d("Unity: ", "in brick.connect");
        comm.open();
    }

    public void disconnect()
    {
        comm.close();
    }

    public void bluetoothConnected()
    {
        isConnected = true;

        nxtManager.sendMessage("throwBluetoothConnection", "true");

        this.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
    }

    // region AsyncTask

    @Override
    protected Void doInBackground(Void... params)
    {
        return null;
    }

}
