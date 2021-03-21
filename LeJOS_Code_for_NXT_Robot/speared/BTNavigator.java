package com.prakhar.speared;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.ArcRotateMoveController;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import lejos.util.PilotProps;

public class BTNavigator {
					
	private ArcRotateMoveController classPilot;
	private OdometryPoseProvider poseProvider;
	private Pose pose = new Pose();
	BTConnection connection;
	static DataInputStream dataInputStream;
	static DataOutputStream dataOutputStream;
	static BTConnection btc;
	
	//for display
	static int onLCDPositionY = 0;

				
	public BTNavigator(final ArcRotateMoveController pilot) {
		classPilot = pilot;
		poseProvider = new OdometryPoseProvider(classPilot);
		classPilot.setTravelSpeed(20);
		classPilot.setRotateSpeed(90);
	}
	
	public static void main(String...strings)  {
		
		while(true) {
			
			connectTheDevice();
			
			try {
			 	
				PilotProps pp = new PilotProps();
		    	pp.loadPersistentValues();
		    	float wheelDiameter = Float.parseFloat(pp.getProperty(PilotProps.KEY_WHEELDIAMETER, "4.96"));
		    	float trackWidth = Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, "13.0"));
		    	RegulatedMotor leftMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_LEFTMOTOR, "B"));
		    	RegulatedMotor rightMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_RIGHTMOTOR, "C"));
		    	boolean reverse = Boolean.parseBoolean(pp.getProperty(PilotProps.KEY_REVERSE,"true"));
		    
		    	DifferentialPilot pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor, reverse);
		      
				//Navigator nav = new Navigator(pilot);
				new BTNavigator(pilot).move();
		    	
//				nav.addWaypoint(new Waypoint(20,0));
//				
//				nav.followPath();
//				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
	}
	
	public void move() throws IOException {
	
		boolean running = true;
		
		do {
			int commandType = (int) dataInputStream.readFloat();
			CommandType command = CommandType.values()[commandType];
			
			LCD.clear();
			onLCDPositionY = 1;
			LCD.drawString(command.toString(), 0, onLCDPositionY++);
			LCD.drawInt(commandType, 0, onLCDPositionY++);
		
			if(command == CommandType.MOVE) {
				moveToDestination();
			}
			else if(command == CommandType.CLAW_UP) {
				moveClawUp();
			}
			else if(command == CommandType.CLAW_DOWN) {
				moveClawDown();
			}
		
			else if(command == CommandType.END) {
				dataInputStream.close();
				dataOutputStream.close();
				btc.close();
				running = false;
				
			}		
			
		}while(running);
		
		
//		
//		int commandInput = dataInputStream.readInt();
//		
//		switch(commandInput) {
//		case INPUT_COMMAND_MOVE:
//			moveToDestination();
//			break;
//		
//		case INPUT_COMMAND_CLAW_UP:
//			moveClawUp();
//			break;
//			
//		case INPUT_COMMAND_CLAW_DOWN:
//			moveClawDown();
//			break;
//		default:
//			LCD.drawString("Unsupported Command Input", 0, 1);
//		}
	}

	private void moveClawDown() {
		// TODO Auto-generated method stub
		Motor.A.setSpeed(45);
		Motor.A.rotate(-90);
	}

	private void moveClawUp() {
		// TODO Auto-generated method stub
		Motor.A.setSpeed(45);
		Motor.A.rotate(90);
		
	}

	private void moveToDestination() throws IOException {
		// TODO Auto-generated method stub
		float x = dataInputStream.readFloat();
		float z = dataInputStream.readFloat();
		
		LCD.drawString("x: "+x, 0, onLCDPositionY++);
		LCD.refresh();
		LCD.drawString("z: "+z, 0, onLCDPositionY++);
		LCD.refresh();
		
		pose = poseProvider.getPose();
		Point destination = new Point(x,z);
		float angle = pose.angleTo(destination);
		
		
		LCD.drawInt((int) pose.getHeading(), 0, onLCDPositionY++);
		
		
		classPilot.rotate(angle - pose.getHeading());
		classPilot.travel(pose.distanceTo(destination));
	}
	
	public static void connectTheDevice() {
		LCD.drawString("This is Speared!", 0, onLCDPositionY++);
		LCD.refresh();
		
		btc = Bluetooth.waitForConnection();
		
		dataInputStream = btc.openDataInputStream();
		dataOutputStream  = btc.openDataOutputStream();
	}
}
