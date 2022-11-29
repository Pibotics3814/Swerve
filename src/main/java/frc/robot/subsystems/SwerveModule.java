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
	public CANSparkMax speedMotor;
	public CANSparkMax angleMotor;
	public CANCoder encoder;
	public PIDController turnPIDController;
	public RelativeEncoder WheelEncoder; 
	double position;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex, boolean angleMotorInverted, boolean wheelMotorInverted ) {

		speedMotor = new CANSparkMax( swerveModIndex + 10, MotorType.kBrushless );
		//speedMotor.setIdleMode(IdleMode.kBrake);
		speedMotor.setInverted( wheelMotorInverted );
		speedMotor.setOpenLoopRampRate( 0.2 );

		angleMotor = new CANSparkMax( swerveModIndex + 20, MotorType.kBrushless );
		angleMotor.setInverted( angleMotorInverted );

		encoder = new CANCoder( swerveModIndex + 1 );

		WheelEncoder = speedMotor.getEncoder();
		//if (wheelMotorInverted)	WheelEncoder.setInverted(true);
		WheelEncoder.setPositionConversionFactor( Constants.drvDistPerPulseRev );

		pidConstants = Constants.SWERVE_PID_CONSTANTS[ swerveModIndex ];
		turnPIDController = new PIDController( pidConstants[ 0 ], pidConstants[ 1 ] ,pidConstants[ 2 ] );

        // Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
        turnPIDController.enableContinuousInput( 0.0, Constants.SWERVE_ENC_CIRC );
		turnPIDController.setTolerance( Constants.SWERVE_PID_TOLERANCE );
	}

	public double getSteerPosition(){
		return encoder.getAbsolutePosition();
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.
		position = getSteerPosition();
		System.out.println( position );

		//*
		final var turnOutput = turnPIDController.calculate( position, angle );
		
		angleMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );
		//*/
		speedMotor.set( speed );
	}

    public void initDefaultCommand() {
			// NOTE: no default command unless running swerve modules seperately
    }
}
