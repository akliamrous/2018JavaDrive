package org.usfirst.frc.team4682.robot;
// ADD NULL VALUE FOR RASBERRY PI DATA -1,-1

import com.kauailabs.navx.frc.AHRS.SerialDataType;
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
import edu.wpi.first.wpilibj.SerialPort;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
@SuppressWarnings("deprecation")
public class Robot extends IterativeRobot {
	Joystick JoyL = new Joystick(0); // Create Joystick for Left
	Joystick JoyR = new Joystick(1); // Create Joystick for Right
	 // Initialize test motor
	
	TalonSRX MD3 = new TalonSRX(3); // Initialize Left motor 3
	TalonSRX MD4 = new TalonSRX(4); // Initialize Right motor 4********************************USE FOR ENCODER
	
	TalonSRX MD2 = new TalonSRX(2); // Initialize Left motor 2 ********************************USE FOR ENCODER
	TalonSRX MD5 = new TalonSRX(5); // Initialize Right motor 5
	
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
	//double JoyLX = 0;
	double JoyRY = 0;
	//double JoyRX = 0;
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
	double  	 GOAL 			 = 0;
	double 		 GMV			 = 0;
	double 		 InDist			 = 0;
	double		 ForLoop		 = 0;
	double 		 ForExt			 = 0;
	double 		 colorVal		 = 0;
	boolean 	 LB3			 = false;
	boolean 	 RB3 		 	 = false;
	boolean 	 Sinput1 		 = SmartDashboard.getBoolean("AUTO LEFT", false);
	boolean 	 Sinput2		 = SmartDashboard.getBoolean("CENTER AUTO", false);
	boolean		 Sinput3		 = SmartDashboard.getBoolean("AUTO RIGHT", false);
	boolean		 Sinput4		 = SmartDashboard.getBoolean("AUTO SWITCH", false);
	boolean		 Sinput5 		 = SmartDashboard.getBoolean("AUTO SCALE", false);
	
	//--------------------------------------------------------------------------------------------------------------------------------------------//
	//Fancynioangs
	
	String  	 gameData;
	PlotThread   plotThread;
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
	AHRS ahrs; // AHRS board
	
	//PlotThread _plotThread; // thread for plotting encoder things
	*/
	
	public void dT(double GMV1,double GMV2,double GMV3, double GMV4)
	    {
	    	MD2.set(ControlMode.PercentOutput, GMV1);//	Set Motors to drive values
			MD5.set(ControlMode.PercentOutput, GMV2);
			MD4.set(ControlMode.PercentOutput, GMV3);
			MD3.set(ControlMode.PercentOutput, GMV4);
	    }
	public void gyro(double Digree)
	    {
		 double rotspeed = (ahrs.getYaw()/(Digree*SmartDashboard.getNumber("kp", 1)))-1;
	    	while(ahrs.getYaw() < Digree)
	    	{
				dT(rotspeed,rotspeed,-rotspeed,-rotspeed);
			}
	    	while(ahrs.getYaw() > Digree)
	    	{
	    	dT(-rotspeed,-rotspeed,rotspeed,rotspeed);
	    	}
	    	dT(0,0,0,0);
	    }   
    public void zero(){
    	ahrs.zeroYaw();
    	System.out.println("Yaw Zeroed");
    }
    public void autoDrive(double goal)
    {
    	if(SmartDashboard.getNumber("posL", 0)/4096 < goal)
    	{
    		double speed = ((-(SmartDashboard.getNumber("posL", 0)/4096)/goal*SmartDashboard.getNumber("kp", 1))+1)
    					  +((-(SmartDashboard.getNumber("posR", 0)/4096)/goal*SmartDashboard.getNumber("kp", 1))+1)/2;
    														//avrages out two wierd numbers(PID stuff)
    		dT(speed,speed,speed,speed);
    	}
    }
    public void autoDepositScale(){
    	double ASpeed = (InDist/SmartDashboard.getNumber("Distance_From_Scale", 7)*SmartDashboard.getNumber("kp", 1))-1;
    	if(ScV == 1 )
    	{
			if(InDist < SmartDashboard.getNumber("Distance_From_Scale", 7))
			{
				
				ForExt = SmartDashboard.getNumber("Wait_Time", 5);
				for(ForLoop = 0; ForLoop < ForExt; ForLoop++)
				{
				MM6.set(ControlMode.PercentOutput, 1);
				MM7.set(ControlMode.PercentOutput, 1);
				MM8.set(ControlMode.PercentOutput, 1);
				MM9.set(ControlMode.PercentOutput, 1);
				Timer.delay(1);
				} 
				MM6.set(ControlMode.PercentOutput, 0);
				MM7.set(ControlMode.PercentOutput, 0);
				MM8.set(ControlMode.PercentOutput, 0);
				MM9.set(ControlMode.PercentOutput, 0);
			}
			else
			{
				dT(ASpeed,ASpeed,ASpeed,ASpeed);
			}
    	}
    }
    public void autoDepositSwitch(){
    	double ASpeed = (InDist/SmartDashboard.getNumber("Distance_From_Scale", 7)*SmartDashboard.getNumber("kp", 1))-1;
    	if(SwV == 1 )
    	{
			if(InDist < SmartDashboard.getNumber("Distance_From_Switch", 7))
			{
				ForExt = SmartDashboard.getNumber("Wait_Time", 5);
				for(ForLoop = 0; ForLoop < ForExt; ForLoop++)
				{
				MM6.set(ControlMode.PercentOutput, 0.2);
				MM7.set(ControlMode.PercentOutput, 0.2);
				MM8.set(ControlMode.PercentOutput, 0.2);
				MM9.set(ControlMode.PercentOutput, 0.2);
				} 
				MM6.set(ControlMode.PercentOutput, 0);
				MM7.set(ControlMode.PercentOutput, 0);
				MM8.set(ControlMode.PercentOutput, 0);
				MM9.set(ControlMode.PercentOutput, 0);
			}
			else
			{
				dT(ASpeed,ASpeed,ASpeed,ASpeed);
			}
    	}
    }
    
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	public Robot() {
	
			try	
			{
			ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
			ahrs.enableLogging(true);
			} 
			catch (RuntimeException ex ) 
			{
	            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
	        }
			Timer.delay(1);
		
	}
	
	
	public void robotInit() {
		
		Scale = NetworkTable.getTable("Scale"); // Initialize Network Table things
		Switch = NetworkTable.getTable("Switch");
		Switch.putNumber("X", 0);
		Switch.putNumber("Y", 0);
		//Switch.putNumber("Z", 0);
		Scale.putNumber("X", 0);
		Scale.putNumber("Y", 0);
		//Scale.putNumber("Z", 0);
		Scale.putNumber("View", 0);
		//Scale.putNumber("Color", 0);
		Switch.putNumber("View", 0);
		//Switch.putNumber("Color", 0);
		SmartDashboard.putNumber("MotorTest", 0);
		//table.putBoolean("bool", false);
		
		


		InDist = Ultra.getVoltage()*512/5;
		SmartDashboard.putBoolean("AUTO LEFT", false);
		SmartDashboard.putBoolean("CENTER AUTO", false);
		SmartDashboard.putBoolean("AUTO RIGHT", false);
		SmartDashboard.putBoolean("AUTO SWITCH", false);
		SmartDashboard.putBoolean("AUTO SCALE", false);
		SmartDashboard.putNumber("kp", 1);
		SmartDashboard.putNumber("Distance_From_Switch", 7);
		SmartDashboard.putNumber("Distance_From_Scale", 7);
		SmartDashboard.putNumber("Wait_Time", 5);
		SmartDashboard.putNumber("Yaw", 0);
		MD2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 10);
		MD2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		MD4.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 10);
		MD4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		plotThread = new PlotThread(this);
		new Thread(plotThread).start();
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
	
		SmartDashboard.putNumber("Yaw", ahrs.getYaw());
		
		if(Sinput1 == true)
		{
			//Auto Left Code
			if(Sinput4 == true)
			{
				//Switch
				if(gameData.charAt(0) == 'L')
				{
					//Left Auto code for switch on left
					System.out.println("Left Pos; Switch Facing Left ; 1");
					autoDrive(10);						//1 Forward
					gyro(90);							//2 gyro = 90
					autoDepositSwitch();				//3 deposit cube 
					System.out.println("Whew... Completed");
				}else{
					//Right Auto code for switch on left
					System.out.println("Left Pos; Switch Facing Right ; 2");
					
					autoDrive(12);						//1 forward
					gyro(90);							//2 gyro = 90
					autoDrive(17);						//3 forward
					gyro(180);							//4 gyro = 180
					autoDrive(3);						//5 forward
					gyro(270);							//6 gyro = 270
					autoDepositSwitch();				//7 deposit cube on switch
					System.out.println("Whew... Completed");
				}
			}else if(Sinput5 == true){
				//Scale
				if(gameData.charAt(1) == 'L'){
					//Left Auto Code for Scale on left
					System.out.println("Left Pos; Scale Facing Left ; 3");
					
					while(colorVal != 1){		//1 Drive Forward
					autoDrive(1);
					}
					autoDrive(1);						//2 wait for color sensor
					gyro(90);					//3 rotate 90
					autoDepositScale();				//4 deposit cube on scale
					
				}else{
												//Right Auto Code for Scale on left
					System.out.println("Left Pos; Scale Facing Right ; 4");
					autoDrive(12);						//1 forward
					gyro(90);					//2 gyro 90
					autoDrive(17);						//3 forward
					gyro(0);					//4 gyro 0
					while(colorVal != 1){	 	//5 forward
					autoDrive(1);
					}
					autoDrive(1);						//6 wait for color sensor
					gyro(-90);					//7 gyro -90
					autoDepositScale();				//8 deposit cube on scale
						
				}
			}
		}else if(Sinput2 != true){
			//Auto Center Code
			if(Switch_or_Scale.get() != true){
				//Switch is on scale is off
				//Switch
				if(gameData.charAt(0) == 'L'){
					//Left Auto code for switch in center
					System.out.println("Center Pos; Switch Facing Left ; 5");
					//possible wait.....
					//
					
				}else{
					//Right Auto code for switch in center
					System.out.println("Center Pos; Switch Facing Right ; 6");
				}
			}else{
				//Scale
				if(gameData.charAt(1) == 'L'){
					//Left Auto code for Scale in center
					System.out.println("Center Pos; Scale Facing Left ; 7");
				}else{
					//Right Auto code for scale in center
					System.out.println("Center Pos; Scale Facing Right ; 8");
				}
			}
		}else if(Sinput3 != true){
			//Auto Right Code
			if(Switch_or_Scale.get() != true){
				//Switch is on scale is off
				//Switch
				if(gameData.charAt(0) == 'L'){
					//Left auto code for switch on right
					System.out.println("Right Pos; Switch; Facing Left ; 9");
					autoDrive(12);//1 forward
					gyro(-90);//2 gyro -90
					autoDrive(17);//3 forward
					gyro(-180);//4 gyro -180
					autoDrive(1);//5 forward
					gyro(-270);//6 gyro -270
					autoDepositSwitch();//7deliver cube
				}else{
					//Right auto code for switch on right
					System.out.println("Right Pos; Switch; Facing Right ; 10");
					autoDrive(10);//1 forward
					gyro(-90);//2 gyro = -90
					autoDepositSwitch();//3diliver cube
				}
			}else{
				//Scale
				if(gameData.charAt(1) == 'L'){
					//Right auto code for scale on right
					System.out.println("Right Pos; Scale; Facing Left ; 11");
					autoDrive(12);//1 forward
					gyro(-90);//2 gyro -90
					autoDrive(17);//3 forward 
					gyro(0);//4 gyro 0
					while(colorVal != 1){	//5 forward
						autoDrive(1);				
					}
					autoDrive(1);//6 untill color line
					gyro(90);//7 gyro 90
					autoDepositScale();//8 deposit cube
				}else{
					// auto code for scale on right
					System.out.println("Right Pos; Scale; Facing Right ; 12");
					//1 forward untill line
					//2 rotate -90
					//3 deposit cube
				}
			}
		}else{
			System.out.println("ERROR 13");
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
		if(JoyL.getZ()>0.5){
			MD2.setSelectedSensorPosition(0, 1, 100);
		}
		SmartDashboard.putNumber("Ultra IN", InDist);

	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
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
				double velocityR = this.robot.MD2.getSelectedSensorPosition(0);
				double velocityL = this.robot.MD4.getSelectedSensorPosition(0);
				SmartDashboard.putNumber("posL", (velocityL/4096));
				SmartDashboard.putNumber("posR", (velocityR/4096));
			}
		}
	}
}
	
