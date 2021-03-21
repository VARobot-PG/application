package com.defaultcompany.nxtbluetoothplugin;

import android.bluetooth.BluetoothSocket;

public interface IConnected
{
    void connected(BluetoothSocket socket);

    void failed();
}
