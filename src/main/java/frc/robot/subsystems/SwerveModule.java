package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
	private final double[] pidConstants;
	public CANSparkMax driveMotor;
	public CANSparkMax steerMotor;
	public CANCoder steerEncoder;
	public PIDController turnPIDController;
	public RelativeEncoder driveEncoder; 
	double position;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex, boolean angleMotorInverted, boolean wheelMotorInverted ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		//speedMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted( wheelMotorInverted );
		driveMotor.setOpenLoopRampRate( 0.2 );

		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		steerMotor.setInverted( angleMotorInverted );

		steerEncoder = new CANCoder( Constants.SWERVE_ENCODER_IDS[ swerveModIndex ] );

		driveEncoder = driveMotor.getEncoder();
		//if (wheelMotorInverted)	WheelEncoder.setInverted(true);
		driveEncoder.setPositionConversionFactor( Constants.drvDistPerPulseRev );

		pidConstants = Constants.SWERVE_PID_CONSTANTS[ swerveModIndex ];
		turnPIDController = new PIDController( pidConstants[ 0 ], pidConstants[ 1 ], pidConstants[ 2 ] );

        // Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
        turnPIDController.enableContinuousInput( 0.0, Constants.SWERVE_ENC_CIRC );
		turnPIDController.setTolerance( Constants.SWERVE_PID_TOLERANCE );
	}

	public double getSteerPosition() {
		return steerEncoder.getAbsolutePosition();
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.
		position = getSteerPosition();
		System.out.println( position );

		//*
		final var turnOutput = turnPIDController.calculate( position, angle );
		
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );
		//*/
		driveMotor.set( speed );
	}

    public void initDefaultCommand() {
			// NOTE: no default command unless running swerve modules seperately
    }
}
