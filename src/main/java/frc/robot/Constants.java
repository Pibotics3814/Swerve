// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveModule;

public final class Constants {
	public static double driveDirection = 1.0;
	public static double JOYSTICK_DEADBAND = 0.2; // was .05
	public static double TRIGGER_DEADBAND = 0.2;
	
	public static double RobotTargetHeading = 0.0;
	public static boolean EnableAutoRobotHeading = false;

	////////////////////////////////////////
	//               Swerve               //
	////////////////////////////////////////

	public static final double[] SWERVE_SETPOINT_OFFSET = { 0.0, 0.0, 0.0, 0.0 }; 
	public static double[][] SWERVE_PID_CONSTANTS = { 
		{ 1.0, 0.0, 0.0 }, 
		{ 1.0, 0.0, 0.0 },
		{ 1.0, 0.0, 0.0 },
		{ 1.0, 0.0, 0.0 } 
	};

	public static boolean[] ANGLE_MOTOR_INVERTED = { false, false, false, false };
	public static boolean[] WHEEL_MOTOR_INVERTED = { true, true, false, false };
	
	public static SwerveModule[] swerveMod;
	
	/*
	 * Swerve constants for swerve module calculations
	 * Don't question and just assume
	 * Want an explanation? Me too...
	 */
	public static double SWERVE_ENC_CIRC = 4.927;
	public static double SWERVE_LENGTH = 21.5;
	public static double SWERVE_WIDTH = 21.5;
	public static double SWERVE_RADIUS = Math.sqrt( Math.pow( SWERVE_LENGTH, 2 ) + Math.pow( SWERVE_WIDTH, 2 ) );
	public static double SWERVE_PID_TOLERANCE = SWERVE_ENC_CIRC / 100.0 / 20.0;
	public static double driveMult = 0.7;
    //TODO: update to match motors
    public static double drvDistPerPulseRev = ( 3.9 * 3.14 ) / ( 42 / 6.75 );  // inches
    //                                      (wheel circum / (encppr * swerve Ratio)

	/*
	 * Swerve module motor and encoder ids
	 */
	public static int[] SWERVE_DRIVE_MOTOR_IDS =     { 10, 11, 12, 13 };
	public static int[] SWERVE_STEER_MOTOR_IDS =     { 20, 21, 22, 23 };
	public static int[] SWERVE_ENCODER_IDS =         {  1,  2,  3,  4 };
}
