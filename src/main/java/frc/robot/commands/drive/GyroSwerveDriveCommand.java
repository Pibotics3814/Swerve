package frc.robot.commands.drive;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GyroSwerveDrive;

public class GyroSwerveDriveCommand extends CommandBase {
    DoubleSupplier dX, dY, dZ;
    double driveX, driveY, driveZ;
    double rawX, rawY, rawZ;
    ADIS16470_IMU m_gyro;
    GyroSwerveDrive m_gyroSwerveDrive;

    public GyroSwerveDriveCommand(DoubleSupplier stick_x, DoubleSupplier stick_y, DoubleSupplier stick_z, ADIS16470_IMU imu, GyroSwerveDrive gyroSwerveDrive) {
        dX = stick_x;
        dY = stick_y;
        dZ = stick_z;
        m_gyro = imu;
        m_gyroSwerveDrive = gyroSwerveDrive;
        addRequirements( m_gyroSwerveDrive );
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      driveX = 0.0;
      rawX = dX.getAsDouble();
      if(Math.abs(rawX) > Constants.JOYSTICK_DEADBAND) driveX = rawX;
      driveY = 0.0;
      rawY = dY.getAsDouble();
      if(Math.abs(rawY) > Constants.JOYSTICK_DEADBAND) driveY = rawY;
      driveZ = 0.0;
      rawX = dZ.getAsDouble();
      if(Math.abs(rawZ) > Constants.JOYSTICK_DEADBAND) driveZ = rawZ;
      m_gyroSwerveDrive.gyroDrive( driveX, driveY, driveZ, Math.toRadians( m_gyro.getAngle() ) );
    }

  
  @Override
  public void end(boolean interrupted) {
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
