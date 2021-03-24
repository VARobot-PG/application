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
					
	private ArcRotateMoveController classPilotForward;
	
	private OdometryPoseProvider poseProviderForward;
	
	private Pose pose = new Pose();
	BTConnection connection;
	static DataInputStream dataInputStream;
	static DataOutputStream dataOutputStream;
	static BTConnection btc;
	
	static BTNavigator backwardNav;
	public static final int FORWARD = 1;
	public static final int BACKWARD = 2;
	
	static DifferentialPilot backPilot;
	static BTNavigator forwardNav;
	//for display
	static int onLCDPositionY = 0;
				
	public BTNavigator(final ArcRotateMoveController pilot, int direction) {
		classPilotForward = pilot;
		poseProviderForward = new OdometryPoseProvider(classPilotForward);
		classPilotForward.setTravelSpeed(20);
		classPilotForward.setRotateSpeed(90);
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
		    
		    	boolean backReverse = Boolean.parseBoolean(pp.getProperty(PilotProps.KEY_REVERSE,"false"));
		    	
		    	DifferentialPilot pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor, reverse);
		    	backPilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor, backReverse);

				//Navigator nav = new Navigator(pilot);
				forwardNav = new BTNavigator(pilot, FORWARD);
				forwardNav.move();
		    
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
			else if(command == CommandType.RESET) {
				setRobotSpecs();
			}
			else if(command == CommandType.BACK) {
				new BTNavigatorBackwards(backPilot).moveBack();
			}
			else if(command == CommandType.LOCATION) {
				sendMyLocation();
			}
			else if(command == CommandType.END) {
				dataInputStream.close();
				dataOutputStream.close();
				btc.close();
				running = false;
				
			}		
			
		}while(running);
		
	}

	private void sendMyLocation() {
		Pose currentPose = poseProviderForward.getPose();
		Point location = currentPose.getLocation();
		
		LCD.drawString("Sending my location", 0, onLCDPositionY++);
		
		try {
			dataOutputStream.writeFloat(location.x);
			//dataOutputStream.writeFloat(location.y);
			LCD.refresh();
			LCD.drawString("x: " + location.x, 0, onLCDPositionY++);
			//LCD.drawString("y: " + location.y, 0, onLCDPositionY++);

		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.println("IO Exception Reading Bytes: ");
			e.printStackTrace();
		}
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
	
	public void setRobotSpecs(){
		LCD.drawString("Setting Robot Specs", 0, onLCDPositionY++);
		
	//	pose = poseProviderForward.getPose();
		pose.setHeading(0);
		
	}
	
	
	
	private void moveToDestination() throws IOException {
		// TODO Auto-generated method stub
		float x = dataInputStream.readFloat();
		float z = dataInputStream.readFloat();
		
		LCD.drawString("x: "+x, 0, onLCDPositionY++);
		LCD.refresh();
		LCD.drawString("z: "+z, 0, onLCDPositionY++);
		LCD.refresh();
		
	
		pose = poseProviderForward.getPose();
		Point destination = new Point(x,z);
		float angle = pose.angleTo(destination);
		
		
		LCD.drawInt((int) pose.getHeading(), 0, onLCDPositionY++);
		
		float angleToRotate = angle - pose.getHeading();
		
		LCD.drawString("To rotate, angle : " + angleToRotate, 0, onLCDPositionY++);
		LCD.drawInt( (int) ((int)angle - pose.getHeading()), 0, onLCDPositionY++);
		
		
		classPilotForward.rotate(angleToRotate);
		classPilotForward.travel(pose.distanceTo(destination));
		
	}
	
	public static void connectTheDevice() {
		LCD.drawString("This is Speared!", 0, onLCDPositionY++);
		LCD.refresh();
		
		btc = Bluetooth.waitForConnection();
		
		dataInputStream = btc.openDataInputStream();
		dataOutputStream  = btc.openDataOutputStream();
	}
}

// NAVIGATOR TO DRIVE THE ROBOT BACKWARDS

class BTNavigatorBackwards{
	private ArcRotateMoveController classPilotBackward;
	private OdometryPoseProvider poseProviderBackward;

	private Pose pose = new Pose();

	
	public BTNavigatorBackwards(final ArcRotateMoveController pilot) {
		classPilotBackward = pilot;
		poseProviderBackward = new OdometryPoseProvider(classPilotBackward);
		classPilotBackward.setTravelSpeed(20);
		classPilotBackward.setRotateSpeed(90);
	}
	
	public void moveBack() {
		try {
			float x = BTNavigator.dataInputStream.readFloat();
			float z = BTNavigator.dataInputStream.readFloat();
			

			LCD.drawString("BACK MOVE " + x + " " + z, 0, BTNavigator.onLCDPositionY++);
			LCD.refresh();
			
			pose = poseProviderBackward.getPose();		
			
			LCD.drawString("MOVE TO " + x + " " + z, 0, BTNavigator.onLCDPositionY++);
			LCD.refresh();
			
			
			Point destination = new Point(x,z);
			
			classPilotBackward.travel(pose.distanceTo(destination));
			
			BTNavigator.forwardNav.move();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}	
	}
}
