package com.defaultcompany.nxtbluetoothplugin;

import android.bluetooth.BluetoothSocket;
import android.os.AsyncTask;


public class BluetoothClient extends AsyncTask<Void, String, Void>
{
    private final BluetoothSocket btSocket;

    private Brick brick;

    public BluetoothClient(Brick brick, BluetoothSocket socket)
    {
        this.brick = brick;

        System.out.println("Initialising Bluetooth Client!");

        // Set the final socket ref
        btSocket = socket;
        System.out.println("Bluetooth Client Initialised!");
    }

    // region AsyncTask


    @Override
    protected Void doInBackground(Void... params)
    {
        loopReceiver();
        return null;
    }

    private void loopReceiver()
    {
        System.out.println("Initialising Bluetooth Receiver!");
    }

    // endregion AsyncTask
}
