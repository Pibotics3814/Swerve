package frc.robot.commands.drive;

import frc.robot.Robot;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.math.MathUtil;

public class GyroSwerveDriveCommand extends CommandBase {


  public GyroSwerveDriveCommand () {
    addRequirements( Robot.gyroSwerveDrive );
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if(Constants.Driverstartbutton) {   // drive controller start button pressed
        Constants.gyro.reset(); //TODO: Add gyro input
        Robot.gyroSwerveDrive.driveStraight( 0.0 );
        Robot.gyroSwerveDrive.reset_encoder();
        }

    double axis0 = 0.0;  // forward motion
    double axis1 = 0.0;  // strafe motion
    int povValue = Constants.DriverPOV;
    if (povValue < 0) {
        axis0 = Constants.driveMult * Constants.DriverYaxis;  // Forward motion
        axis1 = Constants.driveMult * Constants.DriverXaxis;  // strafe motion
    }
	else {
	    switch (povValue) {
			case 0: 
				axis1 = -1.0;
                axis0 = 0.0;
				break;
			case 45: 
                axis1 = -0.7;
                axis0 = 0.7;
                break;
            case 90: 
                axis1 = 0.0;
                axis0 = 1.0;
                break;
            case 135: 
                axis1 = 0.7;
                axis0 = 0.7;
                break;
            case 180: 
                axis1 = 1.0;
                axis0 = 0.0;
                break;
            case 225: 
                axis1 = 0.7;
                axis0 = -0.7;
                break;
            case 270: 
                axis1 = 0.0;
                axis0 = -1.0;
                break;
            case 315: 
                axis1 = -0.7;
                axis0 = -0.7;
                break;
            default: ;//do nothing
        };
        axis0 *= Constants.driveMult;  
        axis1 *= Constants.driveMult;  
      }
      
    //TODO: Add driver inputs
   double axis4 = Constants.driveMult * Robot.m_oi.driveController.getRawAxis( 4 );  // rotational motion
   double axis5 = Constants.driveMult * Robot.m_oi.driveController.getRawAxis( 5 );

    double Rad1 = Math.sqrt( Math.pow( axis0, 2 ) + Math.pow( axis1, 2 ) );
    double Rad2 = Math.sqrt( Math.pow( axis4, 2 ) + Math.pow( axis5, 2 ) );
    if (Rad1 < Constants.JOYSTICK_DEADBAND) { axis0 = 0.0; axis1 = 0.0; }
    if (Rad2 < Constants.JOYSTICK_DEADBAND) { axis4 = 0.0; axis5 = 0.0; }


    double rotMult = 1.0;
    double mult = 1;
    //the driver left trigger can be used to reduce the joystick forward and rotate multiplier by a factor 1 to .5
    if (Constants.DriverLeftTrigger > 0.0) {
        rotMult = 0.2;
        mult = 0.2;
        }
    if(axis0 != 0 || axis1 != 0 || axis4 != 0) Robot.gyroSwerveDrive.gyroDrive( axis0 * mult, axis1 * mult, axis4 * rotMult );
        // otherwise 
    else Robot.gyroSwerveDrive.gyroDriveAngle();  // keeps the wheels at the last angle
  }

  
  @Override
  public void end() {
  }

  @Override
  public void interrupted() {
  }
  

  @Override
  public boolean isFinished() {
    return false;
  }
}
