// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveModule;

public final class Constants {
	public static double driveDirection = 1.0;
	public static double JOYSTICK_DEADBAND = 0.2; // was .05
	public static double TRIGGER_DEADBAND = 0.2;

	public static boolean FieldCentricDrive = true;
	
	public static double RobotTargetHeading = 0.0;
	public static boolean EnableAutoRobotHeading = false;

	// joy stick values
	public static double  OperatorLeftTrigger = 0.;
	public static double  OperatorRightTrigger = 0.;
	public static double  DriverLeftTrigger = 0.;
	public static double  DriverRightTrigger = 0.;
	public static int     OperatorPOV = -1;
	public static int     DriverPOV = -1;
	public static boolean Driverbackbutton = false;
	public static double  OperatorArmJoystick = 0.0;
	public static double  OperatorClimberJoystick = 0.0;
	public static double  DriverYaxis = 0.;
	public static double  DriverXaxis = 0.;
	public static boolean Driverstartbutton = false;

	public static int driverStick = 0;

	////////////////////////////////////////
	//               Swerve               //
	////////////////////////////////////////

	public static final double[] SWERVE_SETPOINT_OFFSET = { 
		0.0, //Front Right
		0.0, //Rear Right
		0.0, //Rear Left
		0.0  //Front Left
	}; 

	public static double[][] SWERVE_STEER_PID_CONSTANTS = { 
		// kP   kI   kD
		{ 0.6, 0.2, 0.0 }, //Front Right
		{ 0.6, 0.2, 0.0 }, //Rear Right
		{ 0.6, 0.2, 0.0 }, //Rear Left
		{ 0.6, 0.2, 0.0 }  //Front Left
	};

	//TODO: Add PID loop for drive motors
	public static double[][] SWERVE_DRIVE_PID_CONSTANTS = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }, //Front Right
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }, //Rear Right
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }, //Rear Left
		{ 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 }  //Front Left
	};

	public static boolean[] STEER_MOTOR_INVERTED = { false, false, false, false };
	public static boolean[] DRIVE_MOTOR_INVERTED = { true, true, false, false };
	
	public static SwerveModule[] swerveMod;
	
	/*
	 * Swerve constants for swerve module calculations
	 * Don't question and just assume
	 * Want an explanation? Me too...
	 */
	public static double SWERVE_ENC_CIRC = 4.927;
	public static double SWERVE_FRAME_LENGTH = 21.5;
	public static double SWERVE_FRAME_WIDTH = 21.5;
	public static double SWERVE_RADIUS = Math.sqrt( Math.pow( SWERVE_FRAME_LENGTH, 2 ) + Math.pow( SWERVE_FRAME_WIDTH, 2 ) );
	public static double SWERVE_PID_TOLERANCE = SWERVE_ENC_CIRC / 100.0 / 20.0;
	public static double driveMult = 0.7;
    //TODO: update to match motors
    public static double drvDistPerPulseRev = ( 3.9 * 3.14 ) / ( 42 / 6.75 );  // inches
    //                                      (wheel circum / (encppr * swerve Ratio)

	/*
	 * Swerve module motor and encoder ids
	 * { Front Right, Back Right, Back Left, Front Left }
	 */
	public static int[] SWERVE_DRIVE_MOTOR_IDS =     { 10, 11, 12, 13 };
	public static int[] SWERVE_STEER_MOTOR_IDS =     { 20, 21, 22, 23 };
	public static int[] SWERVE_ENCODER_IDS =         {  1,  2,  3,  4 };

	private static int swerveModuleNumber = 4;

	public static void init() {
		// Front Right = 0, Back Right = 1, Back Left = 2, Front Left = 3
		//*
		swerveMod = new SwerveModule[swerveModuleNumber];
		for (int i = 0; i < swerveModuleNumber; i++) {
			 swerveMod[i] = new SwerveModule( i );
		}
		//*/
	}
}
