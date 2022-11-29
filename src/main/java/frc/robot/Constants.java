// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveModule;

public final class Constants {
    public static double kP = 1.0;
	public static double SWERVE_ENC_CIRC = 4.927;
	public static final double[] SWERVE_SETPOINT_OFFSET = {0., 0., 0.,0.}; 
	public static double[][] SWERVE_PID_CONSTANTS = {{kP, 1.0, 0.0}, {kP, 0.0, 0}, {kP, 0.0, 0}, {kP, 0.0, 0}};
	public static boolean[] ANGLE_MOTOR_INVERTED = {false, false, false, false};
	public static boolean[] WHEEL_MOTOR_INVERTED = {true, true, false, false};
	public static double driveDirection = 1.;
	public static double JOYSTICK_DEADBAND = 0.2; // was .05
	public static double TRIGGER_DEADBAND = 0.2;
	public static double RobotTargetHeading = 0.;
	public static boolean EnableAutoRobotHeading = false;
	public static SwerveModule[] swerveMod;
	public static double SWERVE_LENGTH = 21.5;
	public static double SWERVE_WIDTH = 21.5;
	public static double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_LENGTH, 2) + Math.pow(SWERVE_WIDTH, 2));
	public static double SWERVE_PID_TOLERANCE = SWERVE_ENC_CIRC / 100.0 / 20.0;
	public static double driveMult = .7;
    //TODO: update to match motors
    public static double drvDistPerPulseRev = ( 3.9 * 3.14 ) / ( 42 / 6.75 );  // inches
    //                                      (wheel circum / (encppr * swerve Ratio)
}
