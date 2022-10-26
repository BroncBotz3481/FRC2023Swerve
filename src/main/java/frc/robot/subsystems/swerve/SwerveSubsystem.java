package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase
{

  public final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
  //Creates 4 New Swerve Modules With their Respective Port Numbers
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveEncoderReversed,
      DriveConstants.kBackLeftTurningEncoderReversed,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveEncoderReversed,
      DriveConstants.kBackRightTurningEncoderReversed,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private final Pigeon2             gyro     = new Pigeon2(DriveConstants.PigeonCANID); //Creates Pigeon2 Gyroscope
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                                                                       new Rotation2d(0));

  //Resets Gryoscope
  public SwerveSubsystem()
  {
    new Thread(() -> { //On a Thread so that rest of code is not blocked
      try
      {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception ignored)
      {

      }
    }).start();
  }

  public void zeroHeading()
  {
    //gyro.reset();     //Need to figure out what to use to reset the gyro on Pigeon2
  }

  //Gets the heading of the Robot
  public double getHeading()
  {
    return gyro.getYaw();
  }

  //Gets Heading in Rotation For robot
  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose()
  {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose)
  {
    odometer.resetPosition(pose, getRotation2d());
  }

  @Override
  public void periodic()
  {
    odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                    backRight.getState());
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  //Stops all modules
  public void stopModules()
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  //Sets Each Module to a state
  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                                                DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //Normalizes Wheel
    // Speeds, Proportionally decreases all wheel speeds to ensure desired effect
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
