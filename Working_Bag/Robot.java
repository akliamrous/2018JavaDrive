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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.RobotBase.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.AnalogInput;
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
	
	TalonSRX ML3 = new TalonSRX(3); // Initialize Left motor 3
	TalonSRX MR4 = new TalonSRX(4); // Initialize Right motor 4********************************USE FOR ENCODER
	
	TalonSRX ML2 = new TalonSRX(2); // Initialize Left motor 2 ********************************USE FOR ENCODER
	TalonSRX MR5 = new TalonSRX(5); // Initialize Right motor 5
	
	TalonSRX MM0 = new TalonSRX(0); // Arm in
	TalonSRX MM1 = new TalonSRX(1); // Arm in ***intake
	
	TalonSRX MM10 = new TalonSRX(10); // Arm in **intake
	TalonSRX MM11 = new TalonSRX(11); // Arm in
	
	TalonSRX MM6 = new TalonSRX(6); // Shooter UL
	TalonSRX MM8 = new TalonSRX(8); // Shooter LL
	
	TalonSRX MM9 = new TalonSRX(9); // Shooter LR
	TalonSRX MM7 = new TalonSRX(7); // Shooter UR
	
	TalonSRX MM12 = new TalonSRX(12); //Belts
	
	//TalonSRX MM13 = new TalonSRX(13); //Belts%%%%%%%%%%%%%%%%%%%%%%%%%%%%Unused
	
	double JoyLY = 0; // Initialize drive variables
	double JoyLX = 0;
	double JoyRY = 0;
	double JoyRX = 0;
	double       LeftSpeed       = 0;                //Initialize driver help and autonomous speeds
    double 		 RightSpeed      = 0;
    
    double 		 SwitchX         = 320;
    double 		 SwitchY         = 0;
    double  	 SwitchZ         = 0;
    double  	 ScaleX          = 320;
    double  	 ScaleY          = 0;
    double 	 	 ScaleZ          = 0;
    double  	 ScV             = 0; 				 // Scale View
	double	  	 ScC             = 0;				 // Scale Color
	double 		 SwV             = 0; 				 // Switch View
	double 		 SwC             = 0;				 // Switch Color
	double		 CV				 = 0;				 // Cube View
	double		 CX				 = 0;				 // Cube X
	double 		 CY 			 = 0;				 // Cube Y
	double   	 ASpeed	     	 = 0;
	double 		 RotSpeed        = 0;
	double 		 WheelRotVal     = 0;
	double  	 GOAL 			 = 0;
	double 		 GMV			 = 0;
	double 		 Position 		 = ML2.getSelectedSensorPosition(0);
	double 		 InDist			 = 0;
	double		 ForLoop		 = 0;
	double 		 ForExt			 = 0;
	double 		 colorVal		 = 0;
	boolean 	 LB3			 = false;
	boolean 	 RB3 		 	 = false;
	
	//--------------------------------------------------------------------------------------------------------------------------------------------//
	//Fancy Things
	String  	 gameData;
	DigitalInput AutoL 		 	 = new DigitalInput(3);
	DigitalInput AutoC 			 = new DigitalInput(2);
	DigitalInput AutoR 			 = new DigitalInput(1);
	DigitalInput Switch_or_Scale = new DigitalInput(4);
	NetworkTable Switch 		 = NetworkTable.getTable("Switch"); //Initialize network Tables Switch, and Scale
    NetworkTable Scale	         = NetworkTable.getTable("Scale"); 
    NetworkTable Cube            = NetworkTable.getTable("Cube"); 
    NetworkTable CS				 = NetworkTable.getTable("ColorSensor"); // posting 1 or 0
    AnalogInput  Ultra 			 = new AnalogInput(0);
	AHRS 		 ahrs;
	/*
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
	boolean LB1 = false;
	boolean RB1 = false;
	boolean RB4 = false;
	boolean RB5 = false;
	boolean LB4 = false;
	boolean LB5 = false;
	boolean Reset = false;
	boolean LB3 = false;
	boolean RB3 = false;
	boolean RB2 = false;
	AnalogInput  Ultra = new AnalogInput(0);
	String gameData;
	DigitalInput AutoL;
	DigitalInput AutoC;
	DigitalInput AutoR;
	DigitalInput Auto0 = new DigitalInput(0);
	DigitalInput Auto1 = new DigitalInput(1);
	DigitalInput Auto2 = new DigitalInput(2);
	DigitalInput Auto3 = new DigitalInput(3);
	DigitalInput Auto4 = new DigitalInput(4);
	DigitalInput Auto5 = new DigitalInput(5);
	DigitalInput Auto6 = new DigitalInput(6);
	DigitalInput Auto7 = new DigitalInput(7);
	DigitalInput Auto8 = new DigitalInput(8);
	DigitalInput Auto9 = new DigitalInput(9);
	//DigitalInput Switch_or_Scale;
	//DigitalInput Ultra = new DigitalInput(9);
	AHRS ahrs; // AHRS board
	
	//PlotThread _plotThread; // thread for plotting encoder things
	*/
	
	 public void dT(double GMV1,double GMV2,double GMV3, double GMV4)
	    {
	    	ML2.set(ControlMode.PercentOutput, GMV1);//	Set Motors to drive values
			MR5.set(ControlMode.PercentOutput, GMV2);
			MR4.set(ControlMode.PercentOutput, GMV3);
			ML3.set(ControlMode.PercentOutput, GMV4);
	    }
	 public void gyro(double Digree)
	    {
	    	while(ahrs.getYaw() < Digree)
	    	{
				dT(WheelRotVal,WheelRotVal,-WheelRotVal,-WheelRotVal);
				WheelRotVal = (ahrs.getYaw()/(Digree*2))-1;
			}
	    }
    
    
    
    
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	public Robot() {
		/*Auto0 = new DigitalInput(0);
		Auto1 = new DigitalInput(1);
		Auto2 = new DigitalInput(2);
		Auto3 = new DigitalInput(3);
		Auto4 = new DigitalInput(4);
		Auto5 = new DigitalInput(5);
		Auto6 = new DigitalInput(6);
		Auto7 = new DigitalInput(7);
		Auto8 = new DigitalInput(8);
		Auto9 = new DigitalInput(9);
		//AutoR = new DigitalInput(6);
		//Switch_or_Scale = new DigitalInput(3);
		*/
		
		
	}
	/*
	public boolean MotorControl(double LeftSpeed1, double LeftSpeed2, double LeftSpeed3, double LeftSpeed4)
	{
		ML2.set(ControlMode.PercentOutput, LeftSpeed1);
		ML3.set(ControlMode.PercentOutput, LeftSpeed2);
		MR4.set(ControlMode.PercentOutput, LeftSpeed3);
		MR5.set(ControlMode.PercentOutput, LeftSpeed4);
		return true;
	}
	*/
	
	public void robotInit() {
		/*
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
		SmartDashboard.putNumber("MotorTest", 0);
		//table.putBoolean("bool", false);
		
		
		WheelRotVal = (ML2.getSelectedSensorPosition(0))/1000;
		
		

*/
	}
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
	//	gameData = DriverStation.getInstance().getGameSpecificMessage();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	/*	AutoL = new DigitalInput(0);
		AutoC = new DigitalInput(1);
		AutoR = new DigitalInput(2);
		while(true){
			SmartDashboard.putBoolean("L?", AutoL.get());
		SmartDashboard.putBoolean("R?", AutoR.get());
		SmartDashboard.putBoolean("C?", AutoC.get());
		}
		*/
		
	
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
		 
	  ML2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1,10);
	  ML2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	  /* fire the plotter 
		//_plotThread = new PlotThread(this);
		//new Thread(_plotThread).start();
	  JoyLY = JoyL.getY();
		JoyLX = JoyL.getX();
		JoyRY = JoyR.getY();
		JoyRX = JoyR.getX();
		RB2 = JoyR.getRawButton(2);
		LB1 = JoyL.getRawButton(1);
		RB1 = JoyR.getRawButton(1);
		LB3 = JoyL.getRawButton(3);
		RB3 = JoyR.getRawButton(3);
		LB4 = JoyL.getRawButton(4);
		RB4 = JoyR.getRawButton(4);
		LB5 = JoyR.getRawButton(5);
		RB5 = JoyR.getRawButton(5);
		*/
		 
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		JoyLY = JoyL.getY();
		JoyRY = JoyR.getY();
		
		
		if(JoyR.getRawButton(3) == true){
			MM12.set(ControlMode.PercentOutput, 1);
		}else{
			MM12.set(ControlMode.PercentOutput, 0);
		}
		if(JoyR.getRawButton(4) == true){
			MM0.set(ControlMode.PercentOutput, 0.4);
		}else if(JoyR.getRawButton(5) == true){
			MM0.set(ControlMode.PercentOutput, -0.4);
		}else{
			MM0.set(ControlMode.PercentOutput, 0);
		}
		if(JoyL.getRawButton(4) == true){
			MM11.set(ControlMode.PercentOutput, 0.4);
		}else if(JoyL.getRawButton(5) == true){
			MM11.set(ControlMode.PercentOutput, -0.4);
		}else{
			MM11.set(ControlMode.PercentOutput, 0);
		}
		
		
		
		if(JoyL.getRawButton(1) == true){
			MM6.set(ControlMode.PercentOutput, 1);
			MM7.set(ControlMode.PercentOutput, -1);
			MM8.set(ControlMode.PercentOutput, -1);
			MM9.set(ControlMode.PercentOutput, 1);
		}else{
			MM6.set(ControlMode.PercentOutput, 0);
			MM7.set(ControlMode.PercentOutput, 0);
			MM8.set(ControlMode.PercentOutput, 0);
			MM9.set(ControlMode.PercentOutput, 0);
		}
		if(JoyR.getRawButton(1) == true){
			MM10.set(ControlMode.PercentOutput, 0.5);
			MM1.set(ControlMode.PercentOutput, -0.5);
		}else{
			MM10.set(ControlMode.PercentOutput, 0);
			MM1.set(ControlMode.PercentOutput, 0);
		}
		//ML2.set(ControlMode.PercentOutput, -JoyLY);
		//MR5.set(ControlMode.PercentOutput, -JoyLY);
		//MR4.set(ControlMode.PercentOutput, JoyRY);
		//ML3.set(ControlMode.PercentOutput, JoyRY);
		//dT(-JoyLY,-JoyLY,JoyRY,JoyRY);
		
		SmartDashboard.putNumber("Ultra IN", Ultra.getVoltage()*512/5);

	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}
	
