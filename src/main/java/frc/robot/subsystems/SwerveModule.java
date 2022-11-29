package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
	public CANSparkMax            driveMotor;
	private SparkMaxPIDController driveVelocityPidController;
	public RelativeEncoder        driveVelocityEncoder; 

	public CANSparkMax            steerMotor;
	public RelativeEncoder        steerVelocityEncoder;
	public CANCoder               steerAngleEncoder;
	private SparkMaxPIDController steerVelocityPidController;
	private PIDController         steerAnglePIDController;

	private final double[]        steerAnglePIDConstants;
	private final double[]        driveVelocityPIDConstants;
	public double                 position;

	public class Vector {
		public Vector(
			double x,
			double y
		) {}
		public double x;
		public double y;
	};
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		//driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotor.setOpenLoopRampRate( 0.2 );
		//TODO: Add PID for driveMotor

		driveVelocityEncoder = driveMotor.getEncoder();
		//if (wheelMotorInverted)	WheelEncoder.setInverted(true);
		driveVelocityEncoder.setPositionConversionFactor( Constants.drvDistPerPulseRev );
		driveVelocityEncoder.setMeasurementPeriod(20);

		driveVelocityPIDConstants = Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex];
		driveVelocityPidController.setP( driveVelocityPIDConstants[0] );
		driveVelocityPidController.setI( driveVelocityPIDConstants[1] );
		driveVelocityPidController.setD( driveVelocityPIDConstants[2] );
		driveVelocityPidController.setIZone( driveVelocityPIDConstants[3] );
		driveVelocityPidController.setFF( driveVelocityPIDConstants[4] );
		driveVelocityPidController.setOutputRange( driveVelocityPIDConstants[5], driveVelocityPIDConstants[6] );

		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );

		steerAngleEncoder = new CANCoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );

		steerAnglePIDConstants = Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex];
		steerAnglePIDController = new PIDController( steerAnglePIDConstants[0], steerAnglePIDConstants[1], steerAnglePIDConstants[2] );

        // Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
        steerAnglePIDController.enableContinuousInput( -1.0, 1.0 );
		steerAnglePIDController.setTolerance( Constants.SWERVE_PID_TOLERANCE );
	}

	public double getSteerAngle() {
		return steerAngleEncoder.getAbsolutePosition();
	}

	public double vectorDotProduct( Vector a, Vector b) {
		return a.x * b.x + a.y * b.y;
	}

	public Vector angleToUnitVector( double angle ) {
		Vector result = new Vector( Math.cos( angle ), Math.sin( angle ) );
		return result;
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.
		double steerAngle = getSteerAngle();
		position = steerAngle < 180.0 ? ( 180.0 - steerAngle ) / -180.0 : ( steerAngle - 180.0 ) / 180.0;
		final var turnOutput = steerAnglePIDController.calculate( position, angle );
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );

		driveMotor.set( speed );
	}

    public void initDefaultCommand() {
			// NOTE: no default command unless running swerve modules seperately
    }
}
