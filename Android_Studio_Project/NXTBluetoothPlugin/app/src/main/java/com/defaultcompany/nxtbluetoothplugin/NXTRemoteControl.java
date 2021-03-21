package com.defaultcompany.nxtbluetoothplugin;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.content.res.Configuration;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.PowerManager;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.Window;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class NXTRemoteControl extends Activity implements OnSharedPreferenceChangeListener {
    
    private boolean NO_BT = false; 
    
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_CONNECT_DEVICE = 2;
    private static final int REQUEST_SETTINGS = 3;
    
    public static final int MESSAGE_TOAST = 1;
    public static final int MESSAGE_STATE_CHANGE = 2;
    
    public static final String TOAST = "toast";
    
    private static final int MODE_BUTTONS = 1;

    
    private BluetoothAdapter mBluetoothAdapter;
    private PowerManager mPowerManager;
    private PowerManager.WakeLock mWakeLock;
    private NXTTalker mNXTTalker;
    
    private int mState = NXTTalker.STATE_NONE;
    private int mSavedState = NXTTalker.STATE_NONE;
    private boolean mNewLaunch = true;
    private String mDeviceAddress = null;
    private TextView mStateDisplay;
    private Button mConnectButton;
    private Button mDisconnectButton;

    private Menu mMenu;
    
    private int mPower = 80;

    private double amod;
    private double lmod;
    private double rmod;
    private int mControlsMode = MODE_BUTTONS;
    
    private boolean mReverse;
    private boolean mReverseLR;
    private boolean mRegulateSpeed;
    private boolean mSynchronizeMotors;


    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //Log.i("NXT", "NXTRemoteControl.onCreate()");
        
        requestWindowFeature(Window.FEATURE_INDETERMINATE_PROGRESS);

        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
        readPreferences(prefs, null);
        prefs.registerOnSharedPreferenceChangeListener(this);
        
        if (savedInstanceState != null) {
            mNewLaunch = false;
            mDeviceAddress = savedInstanceState.getString("device_address");
            if (mDeviceAddress != null) {
                mSavedState = NXTTalker.STATE_CONNECTED;
            }
            
            if (savedInstanceState.containsKey("power")) {
                mPower = savedInstanceState.getInt("power");
            }
            if (savedInstanceState.containsKey("controls_mode")) {
                mControlsMode = savedInstanceState.getInt("controls_mode");
            }
        }
        
        mPowerManager = (PowerManager) getSystemService(Context.POWER_SERVICE);
        mWakeLock = mPowerManager.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK | PowerManager.ON_AFTER_RELEASE, "NXT Remote Control");
        
        if (!NO_BT) {
            mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
       
            if (mBluetoothAdapter == null) {
                Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
                finish();
                return;
            }
        }

        mNXTTalker = new NXTTalker(mHandler);
    }


    public void moveNxtRobot(double l , double r , double a){

        Log.d("Unity:","In nxt remote control move method");
        amod = a;
        lmod = l;
        rmod = r;


        byte power = (byte) mPower;

        byte a_in = (byte) (power*amod);
        byte l_in = (byte) (power*lmod);
        byte r_in = (byte) (power*rmod);

        mNXTTalker.motors3(l_in , r_in, a_in , mRegulateSpeed, mSynchronizeMotors);
    }

    private class DirectionButtonOnTouchListener implements OnTouchListener {

        private double amod;
        private double lmod;
        private double rmod;
        
        public DirectionButtonOnTouchListener(double l, double r, double a) {
            amod = a;
            lmod = l;
            rmod = r;

        }
        
        @Override
        public boolean onTouch(View v, MotionEvent event) {
            //Log.i("NXT", "onTouch event: " + Integer.toString(event.getAction()));
            int action = event.getAction();
            //if ((action == MotionEvent.ACTION_DOWN) || (action == MotionEvent.ACTION_MOVE)) {
            if (action == MotionEvent.ACTION_DOWN) {
                byte power = (byte) mPower;
                if (mReverse) {
                    power *= -1;
                }
                byte a = (byte) (power*amod);
                byte l = (byte) (power*lmod);
                byte r = (byte) (power*rmod);
                if (!mReverseLR) {
                    mNXTTalker.motors3(l, r, a, mRegulateSpeed, mSynchronizeMotors);
                } else {
                    mNXTTalker.motors3(r, l, a, mRegulateSpeed, mSynchronizeMotors);
                }
            } else if ((action == MotionEvent.ACTION_UP) || (action == MotionEvent.ACTION_CANCEL)) {
                mNXTTalker.motors3((byte) 0, (byte) 0, (byte) 0, mRegulateSpeed, mSynchronizeMotors);
            }
            return true;
        }
    }


        
//        mStateDisplay = (TextView) findViewById(R.id.state_display);
//
//        mConnectButton = (Button) findViewById(R.id.connect_button);

        
//        mDisconnectButton = (Button) findViewById(R.id.disconnect_button);
//        mDisconnectButton.setOnClickListener(new OnClickListener() {
//            @Override
//            public void onClick(View v) {
//                mNXTTalker.stop();
//            }
//        });
        
/*
        CheckBox reverseCheckBox = (CheckBox) findViewById(R.id.reverse_checkbox);
        reverseCheckBox.setChecked(mReverse);
        reverseCheckBox.setOnCheckedChangeListener(new OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView,
                    boolean isChecked) {
                mReverse = isChecked;
            }
        });
*/

    @Override
    protected void onStart() {
        super.onStart();
        //Log.i("NXT", "NXTRemoteControl.onStart()");
        if (!NO_BT) {
            if (!mBluetoothAdapter.isEnabled()) {
                Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
            } else {
                if (mSavedState == NXTTalker.STATE_CONNECTED) {
                    BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(mDeviceAddress);
                    mNXTTalker.connect(device);
                } else {
                    if (mNewLaunch) {
                        mNewLaunch = false;
                        findBrick();
                    }
                }
            }
        }
    }

    private void findBrick() {
//        Intent intent = new Intent(this, ChooseDeviceActivity.class);
//        startActivityForResult(intent, REQUEST_CONNECT_DEVICE);
    }
  
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
        case REQUEST_ENABLE_BT:
            if (resultCode == Activity.RESULT_OK) {
                findBrick();
            } else {
                Toast.makeText(this, "Bluetooth not enabled, exiting.", Toast.LENGTH_LONG).show();
                finish();
            }
            break;
        case REQUEST_CONNECT_DEVICE:
            if (resultCode == Activity.RESULT_OK) {
//                String address = data.getExtras().getString(ChooseDeviceActivity.EXTRA_DEVICE_ADDRESS);
//                BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
                //Toast.makeText(this, address, Toast.LENGTH_LONG).show();
//                mDeviceAddress = address;
//                mNXTTalker.connect(device);
            }
            break;
        case REQUEST_SETTINGS:
            //XXX?
            break;
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
        //Log.i("NXT", "NXTRemoteControl.onSaveInstanceState()");
        if (mState == NXTTalker.STATE_CONNECTED) {
            outState.putString("device_address", mDeviceAddress);
        }
        //outState.putBoolean("reverse", mReverse);
        outState.putInt("power", mPower);
        outState.putInt("controls_mode", mControlsMode);
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        //Log.i("NXT", "NXTRemoteControl.onConfigurationChanged()");

    }
    
    private void displayState() {
        String stateText = null;
        int color = 0;
        switch (mState){ 
        case NXTTalker.STATE_NONE:
            stateText = "Not connected";
            color = 0xffff0000;
            mConnectButton.setVisibility(View.VISIBLE);
            mDisconnectButton.setVisibility(View.GONE);
            setProgressBarIndeterminateVisibility(false);
            if (mWakeLock.isHeld()) {
                mWakeLock.release();
            }
            break;
        case NXTTalker.STATE_CONNECTING:
            stateText = "Connecting...";
            color = 0xffffff00;
            mConnectButton.setVisibility(View.GONE);
            mDisconnectButton.setVisibility(View.GONE);
            setProgressBarIndeterminateVisibility(true);
            if (!mWakeLock.isHeld()) {
                mWakeLock.acquire();
            }
            break;
        case NXTTalker.STATE_CONNECTED:
            stateText = "Connected";
            color = 0xff00ff00;
            mConnectButton.setVisibility(View.GONE);
            mDisconnectButton.setVisibility(View.VISIBLE);
            setProgressBarIndeterminateVisibility(false);
            if (!mWakeLock.isHeld()) {
                mWakeLock.acquire();
            }
            break;
        }
        mStateDisplay.setText(stateText);
        mStateDisplay.setTextColor(color);
    }

    private final Handler mHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
            case MESSAGE_TOAST:
                Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST), Toast.LENGTH_SHORT).show();
                break;
            case MESSAGE_STATE_CHANGE:
                mState = msg.arg1;
                displayState();
                break;
            }
        }
    };

    @Override
    protected void onStop() {
        super.onStop();
        //Log.i("NXT", "NXTRemoteControl.onStop()");
        mSavedState = mState;
        mNXTTalker.stop();
        if (mWakeLock.isHeld()) {
            mWakeLock.release();
        }
    }

    @Override
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        readPreferences(sharedPreferences, key);
    }
    
    private void readPreferences(SharedPreferences prefs, String key) {
        if (key == null) {
            mReverse = prefs.getBoolean("PREF_SWAP_FWDREV", false);
            mReverseLR = prefs.getBoolean("PREF_SWAP_LEFTRIGHT", false);
            mRegulateSpeed = prefs.getBoolean("PREF_REG_SPEED", false);
            mSynchronizeMotors = prefs.getBoolean("PREF_REG_SYNC", false);
            if (!mRegulateSpeed) {
                mSynchronizeMotors = false;
            }
        } else if (key.equals("PREF_SWAP_FWDREV")) {
            mReverse = prefs.getBoolean("PREF_SWAP_FWDREV", false);
        } else if (key.equals("PREF_SWAP_LEFTRIGHT")) {
            mReverseLR = prefs.getBoolean("PREF_SWAP_LEFTRIGHT", false);
        } else if (key.equals("PREF_REG_SPEED")) {
            mRegulateSpeed = prefs.getBoolean("PREF_REG_SPEED", false);
            if (!mRegulateSpeed) {
                mSynchronizeMotors = false;
            }
        } else if (key.equals("PREF_REG_SYNC")) {
            mSynchronizeMotors = prefs.getBoolean("PREF_REG_SYNC", false);
        }
    }
}
