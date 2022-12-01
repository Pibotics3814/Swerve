package frc.robot.commands.drive;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GyroSwerveDrive;

public class GyroSwerveDriveCommand extends CommandBase {
    DoubleSupplier dX, dY, dZ;
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
       m_gyroSwerveDrive.gyroDrive( dX.getAsDouble(), dY.getAsDouble(), dZ.getAsDouble(), Math.toRadians( m_gyro.getAngle() ) );
       SmartDashboard.putNumber( "Module 2 encoder", Constants.swerveMod[1].getSteerAngle() );
       SmartDashboard.putNumber( "Module 3 encoder", Constants.swerveMod[2].getSteerAngle() );
       SmartDashboard.putNumber( "Module 4 encoder", Constants.swerveMod[3].getSteerAngle() );
    }

  
  @Override
  public void end(boolean interrupted) {
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
