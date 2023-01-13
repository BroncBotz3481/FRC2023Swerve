package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleMotorType;

public class SwerveSubsystem extends SubsystemBase
{

  private final Timer                                 syncTimer = new Timer();
  public        SwerveDrive<CANSparkMax, CANSparkMax> m_drive;
  //Creates Pigeon2 Gyroscope

  public SwerveSubsystem()
  {
    syncTimer.start();
    SwerveModule<CANSparkMax, CANSparkMax, CANCoder> m_frontRight, m_frontLeft, m_backRight, m_backLeft;
    CANSparkMax fld = new CANSparkMax(DriveConstants.kFrontLeftDriveMotorPort,
                                      MotorType.kBrushless);
    CANSparkMax flt = new CANSparkMax(DriveConstants.kFrontLeftTurningMotorPort,
                                      MotorType.kBrushless);
    CANCoder flc = new CANCoder(DriveConstants.kFrontLeftAbsoluteEncoderPort);
    m_frontLeft = new SwerveModule<>(fld,
                                     flt,
                                     flc,
                                     SwerveModule.SwerveModuleLocation.FrontLeft,
                                     ModuleConstants.kDriveMotorGearRatio,
                                     ModuleConstants.kTurningMotorGearRatio,
                                     DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
                                     Units.inchesToMeters(4),
                                     DriveConstants.kWheelBase,
                                     DriveConstants.kTrackWidth,
                                     DriveConstants.kFreeSpeedRpm,
                                     DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                                     DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    m_frontRight = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kFrontRightAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.FrontRight,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth, DriveConstants.kFreeSpeedRpm,
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    m_backLeft = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kBackLeftDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kBackLeftTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kBackLeftAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.BackLeft,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth, DriveConstants.kFreeSpeedRpm,
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    m_backRight = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kBackRightDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kBackRightTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kBackRightAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.BackRight,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth, DriveConstants.kFreeSpeedRpm,
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    m_drive = new SwerveDrive<>(m_frontLeft,
                                m_backLeft,
                                m_frontRight,
                                m_backRight,
                                new WPI_Pigeon2(DriveConstants.PigeonCANID),
                                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond,
                                false);

    m_drive.zeroGyro();
    m_drive.setDeadband(0.5);
    m_drive.setPIDF(0.07, 0, 0.3, 0, 100, SwerveModuleMotorType.TURNING); // TODO: Change PIDF here.
    m_drive.setAngleDeadband(5);

  }

  /**
   * Drive function
   *
   * @param x                X value
   * @param y                Y value
   * @param r                R value
   * @param fieldOrientation Field oriented drive.
   */
  public void drive(double x, double y, double r, boolean fieldOrientation)
  {
    m_drive.drive(x, y, r, fieldOrientation);
  }

  public void stop()
  {
    m_drive.stopMotor();
  }

  @Override
  public void periodic()
  {
    if (syncTimer.advanceIfElapsed(1))
    {
      m_drive.synchronizeEncoders();
    }
  }
}
