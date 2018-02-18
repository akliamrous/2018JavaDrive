package org.usfirst.frc.team4682.robot;
// ADD NULL VALUE FOR RASBERRY PI DATA -1,-1

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Joystick JoyL = new Joystick(0); // Create Joystick for Left
	Joystick JoyR = new Joystick(1); // Create Joystick for Right
	 // Initialize test motor
	TalonSRX MM0 = new TalonSRX(0); //ACT ramp_________OR_________unused
	TalonSRX MM1 = new TalonSRX(1); //ACT ramp_________OR_________unused
	TalonSRX ML2 = new TalonSRX(2); // Initialize Left motor 2 ********************************USE FOR ENCODER
	TalonSRX ML3 = new TalonSRX(3); // Initialize Left motor 3
	TalonSRX MR4 = new TalonSRX(4); // Initialize Right motor 4********************************USE FOR ENCODER
	TalonSRX MR5 = new TalonSRX(5); // Initialize Right motor 5
	TalonSRX MM6 = new TalonSRX(6); // Shooter UL
	TalonSRX MM7 = new TalonSRX(7); // Shooter UR
	TalonSRX MM8 = new TalonSRX(8); // Shooter LL
	TalonSRX MM9 = new TalonSRX(9); // Shooter LR
	TalonSRX MM10 = new TalonSRX(10); // Arm in
	TalonSRX MM11 = new TalonSRX(11); // Arm in
	TalonSRX MM12 = new TalonSRX(12); //Belts
	TalonSRX MM13 = new TalonSRX(13); //Belts
	double JoyLY = 0; // Initialize drive variables
	double JoyLX = 0;
	double JoyRY = 0;
	double JoyRX = 0;
    NetworkTable Switch; //Initialize network Tables Switch, and Scale
    NetworkTable Scale;
    double LeftSpeed = 0; //Initialize driver help and autonomous speeds
    double RightSpeed = 0;
    double SwitchX = 320;
    double SwitchY = 0;
    double SwitchZ = 0;
    double ScaleX = 320;
    double ScaleY = 0;
    double ScaleZ = 0;
    double ScV = 0;
	double ScC = 0;
	double SwV = 0;
	double SwC = 0;
	double DefaultVal = -1;
	double RotSpeed;
	double WheelRotVal;
	double autoSpeed;
	double GOAL;
	boolean Reset = false;
	boolean LB3 = false;
	boolean RB3 = false;
	String gameData;
	DigitalInput AutoL = new DigitalInput(3);
	DigitalInput AutoC = new DigitalInput(2);
	DigitalInput AutoR = new DigitalInput(1);
	DigitalInput Switch_or_Scale = new DigitalInput(4);
	AHRS ahrs; // AHRS board
	PlotThread _plotThread; // thread for plotting encoder things
	
	
	
    
    
    
    
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	public Robot() {
		
		
	}
	
	public boolean MotorControl(double LeftSpeed1, double LeftSpeed2, double LeftSpeed3, double LeftSpeed4)
	{
		ML2.set(ControlMode.PercentOutput, LeftSpeed1);
		ML3.set(ControlMode.PercentOutput, LeftSpeed2);
		MR4.set(ControlMode.PercentOutput, LeftSpeed3);
		MR5.set(ControlMode.PercentOutput, LeftSpeed4);
		return true;
	}
	
	
	public void robotInit() {
		Scale = NetworkTable.getTable("Scale"); // Initialize Network Table things
		Switch = NetworkTable.getTable("Switch");
		Switch.putNumber("X", 0);
		Switch.putNumber("Y", 0);
		Switch.putNumber("Z", 0);
		Scale.putNumber("X", 0);
		Scale.putNumber("Y", 0);
		Scale.putNumber("Z", 0);
		Scale.putNumber("View", 0);
		Scale.putNumber("Color", 0);
		Switch.putNumber("View", 0);
		Switch.putNumber("Color", 0);
		
		//table.putBoolean("bool", false);
		
		
		WheelRotVal = (ML2.getSelectedSensorPosition(0))/1000;
		System.out.println(WheelRotVal);
		
		


	}
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		/*
		 * Use quararion X,Y, and Z
		 * these are rotational values
		 */
		if(AutoL.get() == true){
			//Auto Left Code
			if(Switch_or_Scale.get() == true){
				//Switch is on scale is off
				//Switch
				if(gameData.charAt(0) == 'L'){
					//Left Auto code for switch on left
					// Drive forward trun 90 digrees and then enable auto tracking
					//Drive Forward "X" # of ratations
					//1 Drive Forward -- USE ENC
					WheelRotVal = (ML2.getSelectedSensorPosition(0))/1000;
					GOAL = 90;
					while(WheelRotVal <= GOAL){
						
						SmartDashboard.putNumber("DistanceVal", WheelRotVal);
						ML2.set(ControlMode.PercentOutput, Math.abs(autoSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(autoSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(autoSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(autoSpeed));
						autoSpeed = (WheelRotVal/GOAL)-1;
					}
						
					//2 Turn 90 -- USE arhs.getyaw
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 enable auto search
					
				}else{
					//Right Auto code for switch on left
					// Drive forward turn 90, drive forward, turn 90, drive forward, turn 90, activee auto trackign
					//1 Drive Forward
					
					//2 Turn 90*
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 drive forward set distance
					//turn 90*
					while(ahrs.getYaw() != 180){
						//goal 180
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//4 drive forward
					
					//5 turn 90*
					while(ahrs.getYaw() != 270){
						//goal 270
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//6 activate tracking
				}
			}else{
				//Scale
				if(gameData.charAt(1) == 'L'){
					//Left Auto Code for Scale on left
					// drive forward read for floor color change, turn 90, activate auto trackign
					//1 Drive Forward
					//2 read for colour change
					//3 turn 90
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//4 auto tracking
				}else{
					//Right Auto Code for Scale on left
					//forward, 90, forward, -90, forard read for color change, -90, activate auto tracking
					//1 Drive Forward
					//2 Rota 90*
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 Drive Forward
					//4 Rota back to 0
					while(ahrs.getYaw() != 0){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//5 forward
					//6 read for clolour change
					//7 rota -90
					while(ahrs.getYaw() != -90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//8 activate auto traking
					
				}
			}
		}else if(AutoC.get() == true){
			//Auto Center Code
			if(Switch_or_Scale.get() == true){
				//Switch is on scale is off
				//Switch
				if(gameData.charAt(0) == 'L'){
					//Left Auto code for switch in center
					//forward, -90, forward, 90, forward, 90, auto track
					//1 forward, 
					//2 set gyro to -90
					while(ahrs.getYaw() != -90){
						//goal -90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 go forward 
					//4 set gyro to 0
					while(ahrs.getYaw() != 0){
						//goal 0
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//5 forward
					//6 set gyro to 90
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//7 turn on auto tracking
				}else{
					//Right Auto code for switch in center
					//forward,90,forward,-90,forward,-90,activate auto tracking
					//1 forward
					//2 set gyro 90
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 forward
					//4 set gyro 0
					while(ahrs.getYaw() != 0){
						//goal 0
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//5 forward
					//6 set gyro to -90
					while(ahrs.getYaw() != -90){
						//goal -90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//7 activate auto tracking for switch
					
					
					
				}
			}else{
				//Scale
				if(gameData.charAt(1) == 'L'){
					//Left Auto code for Scale in center
					//forward -90 forward 90 forward, look for line, 90, auto track
					ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
					ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
					MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
					MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
					RotSpeed = (ahrs.getYaw()/180)-1;
				}else{
					//Right Auto code for scale in center
					//forward 90 forward-90 forward, look for line, -90, auto track
					ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
					ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
					MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
					MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
					RotSpeed = (ahrs.getYaw()/180)-1;
				}
			}
		}else if(AutoR.get() == true){
			//Auto Right Code
			if(Switch_or_Scale.get() == true){
				//Switch is on scale is off
				//Switch
				if(gameData.charAt(0) == 'L'){
					//Right auto code for switch on left
					//forward 90 forward 90 forward 90, activate auto
					//1 Forward
					//2 set gyro to -90
					while(ahrs.getYaw() != -90){
						//goal -90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 Forward
					//4 set gyro to -180
					while(ahrs.getYaw() != -180){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//5 Forward
					//6 Set Gyro to -270
					while(ahrs.getYaw() != -270){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//7 active auto 
				}else{
					//Right auto code for switch on right
					//forward, 90, auto track
					//1 Forward
					//2 Set gyro -90
					while(ahrs.getYaw() != -90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 Auto tracking
				}
			}else{
				//Scale
				if(gameData.charAt(1) == 'L'){
					//Right auto code for scale on left
					// forward -90 forward 90 forward look for color change, 90, activate auto
					//1 Forward
					//2 Set Gyro to -90
					while(ahrs.getYaw() != -90){
						//goal -90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//3 Forward
					//4 set gyro to 0
					while(ahrs.getYaw() != 0){
						//goal 0
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//5 Forward
					//6 continue untill color change
					//7 set gyro to 90
					while(ahrs.getYaw() != 90){
						//goal 90
						//ahrs.getYaw convert to motor value
						ML2.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(RotSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(RotSpeed));
						RotSpeed = (ahrs.getYaw()/180)-1;
					}
					//8 activate gyro 
				}else{
					//Right auto code for scale on right
					//forward look for color change, -90
					//1 forward
					//2 Look fo rcolor change
					//3 set gyro to -90
				}
			}
		}else{
			System.out.println("ERROR");
			//Possible change to auto backfire and only drive over the line,
		}
		
		if(gameData.charAt(0) == 'L'){
			// put Left auto code here
		}else{
			//Put right auto code here
		}
		
		
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		/*
		 * new frame every 1ms, since this is a test project use up as much
		 * bandwidth as possible for the purpose of this test.
		 */
	  ML2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1,10);
	  ML2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	  /* fire the plotter */
		_plotThread = new PlotThread(this);
		new Thread(_plotThread).start();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@SuppressWarnings("deprecation")
	@Override
	public void teleopPeriodic() {
		RB3 = JoyR.getRawButton(3);
		LB3 = JoyL.getRawButton(3);
	SwitchX = Switch.getNumber("X", 0);
	//SwitchY = Switch.getNumber("Y", 0);
	//SwitchZ = Switch.getNumber("Z", 0);
	ScaleX = Scale.getNumber("X", 0);
	//ScaleY = Scale.getNumber("Y", 0);
	//ScaleZ = Scale.getNumber("Z", 0); 
	ScV = /*JoyL.getRawButton(1);*/Scale.getNumber("View", 0); //Scale View
	ScC = Scale.getNumber("Color", 0); //Scale Color
	SwV = /*JoyL.getRawButton(2);*/Switch.getNumber("View", 0); //Switch View
	SwC = Switch.getNumber("Color", 0); //Switch Color
	JoyLY = -JoyL.getY();// Drive Train
	JoyRY = -JoyR.getY();   
	Reset = JoyL.getRawButton(4);
	SmartDashboard.putBoolean("LB4", Reset);
	SmartDashboard.putBoolean("LB3", LB3);
	SmartDashboard.putBoolean("RB3", RB3);
	
	
	System.out.println(ScaleX);
	
	/*
			System.out.print("areas: ");
			for (double area : areas){
				System.out.print(area + " ");
			}
				
			
			System.out.println();
			
		//table.putNumber("test", 1);
		//table.getNumber("test", 0);
		//if(table.getNumber("Motor", 4) == 1){
		//	MotorL3.set(ControlMode.PercentOutput, 50);
		//}else{
		//	MotorL3.set(ControlMode.PercentOutput, 0);
		//}
		 */
		if (RB3 == true){
			if (ScV == 1){
				if(ScaleX != DefaultVal || ScaleY != DefaultVal ){
					if(ScaleX < 315){
						//Rotate Clockwise
						//Left Forward Right back
						//MotorControl(Math.abs(LeftSpeed), Math.abs(LeftSpeed), -Math.abs(LeftSpeed), -Math.abs(LeftSpeed));
						ML2.set(ControlMode.PercentOutput, Math.abs(LeftSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(LeftSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(LeftSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(LeftSpeed));
						LeftSpeed = (ScaleX/320)-1;
						System.out.println(LeftSpeed + "; Clockwise");
					
					} else if(ScaleX > 325){
						//Rotate Counter Clockwise
						//Left Backward Right Forward
						//MotorControl(-Math.abs(RightSpeed), -Math.abs(RightSpeed), Math.abs(RightSpeed), Math.abs(RightSpeed));
						ML2.set(ControlMode.PercentOutput, -Math.abs(RightSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RightSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RightSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RightSpeed));
						RightSpeed =(ScaleX/320)-1;
						System.out.println(RightSpeed + "; Counter Clockwise");
					
					}else{
						System.out.println("Scale not in view");
					}
				}
				
			}
		}else if(LB3 == true){
			if(SwV == 1){
				if(ScaleX != DefaultVal || ScaleY != DefaultVal ){
					if(SwitchX < 315){
						//Rotate Clockwise
						//Left Forward Right back
						//MotorControl(Math.abs(LeftSpeed), Math.abs(LeftSpeed), -Math.abs(LeftSpeed), -Math.abs(LeftSpeed));
						ML2.set(ControlMode.PercentOutput, Math.abs(LeftSpeed));
						ML3.set(ControlMode.PercentOutput, Math.abs(LeftSpeed));
						MR4.set(ControlMode.PercentOutput, -Math.abs(LeftSpeed));
						MR5.set(ControlMode.PercentOutput, -Math.abs(LeftSpeed));
						LeftSpeed = (SwitchX/320)-1;
						System.out.println(LeftSpeed + "; Clockwise");
						
						
					} else if(SwitchX > 325){
						//Rotate Counter Clockwise
						//Left Backward Right Forward
						//MotorControl(-Math.abs(RightSpeed), -Math.abs(RightSpeed), Math.abs(RightSpeed), Math.abs(RightSpeed));
						ML2.set(ControlMode.PercentOutput, -Math.abs(RightSpeed));
						ML3.set(ControlMode.PercentOutput, -Math.abs(RightSpeed));
						MR4.set(ControlMode.PercentOutput, Math.abs(RightSpeed));
						MR5.set(ControlMode.PercentOutput, Math.abs(RightSpeed));
						RightSpeed = (SwitchX/320)-1;
						System.out.println(RightSpeed + "; Counter Clock");
					}	
				}
			}
			}else{
				//Mission	
				
				
				
				
				//Drive
				ML2.set(ControlMode.PercentOutput, JoyLY);//	Set Motors to drive values
				ML3.set(ControlMode.PercentOutput, JoyLY);
				MR4.set(ControlMode.PercentOutput, JoyRY);
				MR5.set(ControlMode.PercentOutput, JoyRY);
				Reset = JoyL.getRawButton(4);
				System.out.println(WheelRotVal);
			}
		}
	
		
		/*if(ScaleZ == TEST){
			
		}else if(ScaleZ <= TEST){
			//forward
			MotorL2.set(ControlMode.PercentOutput, 0.1);
			MotorL3.set(ControlMode.PercentOutput, -0.1);
			MotorR4.set(ControlMode.PercentOutput, 0.1);
			MotorR5.set(ControlMode.PercentOutput, 0.1);
		}else if(ScaleZ >= TEST){
			//backward
			MotorL2.set(ControlMode.PercentOutput, -0.1);
			MotorL3.set(ControlMode.PercentOutput, 0.1);
			MotorR4.set(ControlMode.PercentOutput, -0.1);
			MotorR5.set(ControlMode.PercentOutput, -0.1);
		}
		*/
		
	
		
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
		SmartDashboard.putBoolean("L?", AutoL.get());
		SmartDashboard.putBoolean("R?", AutoR.get());
		SmartDashboard.putBoolean("C?", AutoC.get());
		
	}
	class PlotThread implements Runnable {
		Robot robot;

		public PlotThread(Robot robot) {
			this.robot = robot;
		}

		public void run() {
			/*
			 * speed up network tables, this is a test project so eat up all of
			 * the network possible for the purpose of this test.
			 */
			// NetworkTable.setUpdateRate(0.010); /* this suggests each time
			// unit is 10ms in the plot */
			
			while (true) {
				/* yield for a ms or so - this is not meant to be accurate */
				try {
					Thread.sleep(1);
				} catch (Exception e) {
				}
				/* grab the last signal update from our 1ms frame update */
				double velocity = this.robot.ML2.getSelectedSensorVelocity(0);
				
				double Position = this.robot.ML2.getSelectedSensorPosition(0);
				
				SmartDashboard.putNumber("vel", velocity);
				SmartDashboard.putNumber("Position", this.robot.WheelRotVal);
			}
		}
	}
}
