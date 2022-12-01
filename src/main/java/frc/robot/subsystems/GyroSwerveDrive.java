package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroSwerveDrive extends SubsystemBase {
  public double[] speed = new double[4];
  public double[] angle = new double[4];
  public boolean  fcd = true;

  ADIS16470_IMU m_gyro;

  public void gyroDrive( double str, double fwd, double rot, double gyroAngle ) {
    
    fcd = Constants.FieldCentricDrive;
    computeSwerveInputs( str, fwd, rot, gyroAngle );
    setSetpoints( rot ); 

    for(int i = 0; i < 4; i++) {
        Constants.swerveMod[i].drive( speed[i], angle[i] );
    }
  }

  public void drive(double str, double fwd, double rot) {
    double a = str - rot * 1.5 * ( Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS );
    double b = str + rot * 1.5 * ( Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS );
    double c = fwd - rot * 1.5 * ( Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS );
    double d = fwd + rot * 1.5 * ( Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS );
    
    speed[1] = Math.sqrt( ( a * a ) + ( d * d ) );
    speed[2] = Math.sqrt( ( a * a ) + ( c * c ) );
    speed[0] = Math.sqrt( ( b * b ) + ( d * d ) );
    speed[3] = Math.sqrt( ( b * b ) + ( c * c ) );

    angle[1] = Math.atan2 ( a, d ) / Math.PI;
    angle[2] = Math.atan2 ( a, c ) / Math.PI;
    angle[0] = Math.atan2 ( b, d ) / Math.PI;
    angle[3] = Math.atan2 ( b, c ) / Math.PI;
    setSetpoints(rot); 
    for (int i = 0; i < 4; i++) {
        Constants.swerveMod[i].drive( speed[i], angle[i] );
    }
  }


  public void autoDrive( double angle, double speed ) {
    for (int i = 0; i < 4; i++) {
      Constants.swerveMod[i].drive( angle, speed );
    }
  }

  public void gyroDriveAngle() {
    // keeps wheels at their current angle when zero forward, side and rotate command
    // if the driver's back button is hit then the wheels are command to 0 angle
    for(int i = 0; i < 4; i++) {
      // when back button is pressed then command wheels to zero 
      if ( Constants.Driverbackbutton ) Robot.gyroSwerveDrive.driveStraight( 0.0 );
      else Constants.swerveMod[i].drive( 0.0, angle[i] );
    }
  }

  public double getOppositeAngle( int index ) {
    double opp = angle[index];
    if( opp < ( Constants.SWERVE_ENC_CIRC / 2 ) ) opp += Constants.SWERVE_ENC_CIRC / 2;
    else opp -= Constants.SWERVE_ENC_CIRC / 2;
    return opp;
  }

  public void computeSwerveInputs( double str, double fwd, double rot, double gyroAngle ) {
    if(fcd){
      double intermediary = fwd * Math.cos( gyroAngle ) + str * Math.sin( gyroAngle );
      str = -fwd * Math.sin( gyroAngle ) + str * Math.cos( gyroAngle );
      fwd = intermediary;
    }

    double a = str - rot * ( Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS );
    double b = str + rot * ( Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS );
    double c = fwd - rot * ( Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS );
    double d = fwd + rot * ( Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS );
    
    speed[1] = Math.sqrt( ( a * a ) + ( d * d ) );
    speed[2] = Math.sqrt( ( a * a ) + ( c * c ) );
    speed[0] = Math.sqrt( ( b * b ) + ( d * d ) );
    speed[3] = Math.sqrt( ( b * b ) + ( c * c ) );

    angle[1] = Math.atan2( a, d ) / Math.PI;
    angle[2] = Math.atan2( a, c ) / Math.PI;
    angle[0] = Math.atan2( b, d ) / Math.PI;
    angle[3] = Math.atan2( b, c ) / Math.PI;
  }

  public void setSetpoints( double rot ) {
    for(int i = 0; i < 4; i++) {
      SmartDashboard.putNumber( "angle: " + i, angle[i] );
      SmartDashboard.putNumber( "speed: " + i, speed[i] );

      double offsetSteerAngle = getOffsetSteerEncoderAngle(i);

      double encCount = Constants.swerveMod[i].steerAngleEncoder.getAbsolutePosition();
      angle[i] = ( angle[i] + 1 ) * Constants.SWERVE_ENC_CIRC / 2 + Constants.SWERVE_SETPOINT_OFFSET[i]; 
      if(angle[i] > Constants.SWERVE_ENC_CIRC) angle[i] -= Constants.SWERVE_ENC_CIRC;

      double degreesBeforeFlip = 90.0;
      if(Math.abs( encCount - angle[i] ) > Constants.SWERVE_ENC_CIRC / 360 * degreesBeforeFlip) {
        angle[i] = getOppositeAngle(i);
        speed[i] *= -1;
        //Constants.swerveMod[i].WheelEncoder.setInverted(Constants.ANGLE_MOTOR_INVERTED[i]==false);
      }
    }
  }

  private double getOffsetSteerEncoderAngle( int i ) {
    double offsetAngle = Constants.swerveMod[i].steerAngleEncoder.getAbsolutePosition() + Constants.SWERVE_SETPOINT_OFFSET[i];
    offsetAngle += ( offsetAngle > 360.0 ) && ( offsetAngle < 0.0 ) ? ( offsetAngle > 360.0 ? -360.0 : 360.0 ) : 0.0;
    double remappedAngle = offsetAngle < 180.0 ? ( 180.0 - offsetAngle ) / -180.0 : ( offsetAngle - 180.0 ) / 180.0;
    return remappedAngle;
  }

  public void reset_encoder(){
      Constants.swerveMod[2].driveVelocityEncoder.setPosition( 0.0 );
  }

  public void WheelToCoast(){
    for(int i = 0; i < 4; i++){
       Constants.swerveMod[i].driveMotor.setIdleMode( IdleMode.kCoast );
    }
  }

  public void WheelToBrake(){
    for(int i = 0; i < 4; i++){
       Constants.swerveMod[i].driveMotor.setIdleMode( IdleMode.kBrake );
    }
  }

  public double GetCurrentDistance() {  // reading wheel 0 only
    double dist =0.0;
        dist = -Constants.swerveMod[2].driveVelocityEncoder.getPosition();
        return( dist );
}


  public void driveStraight( double fwd ) {
    double a = 0;
    double b = 0;
    double c = Constants.driveDirection * -fwd;
    double d = Constants.driveDirection * -fwd;
    
    speed[1] = Math.sqrt( d * d );
    speed[2] = Math.sqrt( c * c );
    speed[0] = Math.sqrt( d * d );
    speed[3] = Math.sqrt( c * c );

    angle[1] = Math.atan2( a, d ) / Math.PI;
    angle[2] = Math.atan2( a, c ) / Math.PI;
    angle[0] = Math.atan2( b, d ) / Math.PI;
    angle[3] = Math.atan2( b, c ) / Math.PI;

    for(int i = 0; i < 4; i++){
      double encCount = Constants.swerveMod[i].steerAngleEncoder.getAbsolutePosition(); 
      angle[i] = ( angle[i] + 1 ) * Constants.SWERVE_ENC_CIRC / 2 + Constants.SWERVE_SETPOINT_OFFSET[i]; 
      if(angle[i] > Constants.SWERVE_ENC_CIRC) angle[i] -= Constants.SWERVE_ENC_CIRC;

      double degreesBeforeFlip = 90.0;
      if(Math.abs( encCount - angle[i] ) > Constants.SWERVE_ENC_CIRC / 360 * degreesBeforeFlip) {
        angle[i] = getOppositeAngle( i );
        speed[i] *= -1;
      }

      Constants.swerveMod[i].drive( speed[i], angle[i] );
    }
  }
}