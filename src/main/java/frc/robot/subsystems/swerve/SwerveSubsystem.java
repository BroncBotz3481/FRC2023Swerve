package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveModule.Verbosity;
import frc.robot.subsystems.swerve.SwerveMotor.ModuleMotorType;

public class SwerveSubsystem extends SubsystemBase
{

  private final Timer       syncTimer = new Timer();
  public        SwerveDrive m_drive;
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

    m_drive = new SwerveDrive(m_frontLeft,
                              m_frontRight,
                              m_backLeft,
                              m_backRight,
                              new WPI_Pigeon2(DriveConstants.PigeonCANID),
                              DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                              DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                              DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                              DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond,
                              false);

//    SwerveModuleConfig<CANSparkMax, CANSparkMax, CANCoder> configs[] = new SwerveModuleConfig[]{
//        new SwerveModuleConfig<>(new CANSparkMax(DriveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless),
//                                 new CANSparkMax(DriveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless),
//                                 new CANCoder(DriveConstants.kFrontLeftAbsoluteEncoderPort),
//                                 DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset, SwerveModuleLocation.FrontLeft),
//
//        new SwerveModuleConfig<>(new CANSparkMax(DriveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless),
//                                 new CANSparkMax(DriveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless),
//                                 new CANCoder(DriveConstants.kFrontRightAbsoluteEncoderPort),
//                                 DriveConstants.kFrontRightDriveAbsoluteEncoderOffset, SwerveModuleLocation
//                                 .FrontRight),
//
//        new SwerveModuleConfig<>(new CANSparkMax(DriveConstants.kBackLeftDriveMotorPort, MotorType.kBrushless),
//                                 new CANSparkMax(DriveConstants.kBackLeftTurningMotorPort, MotorType.kBrushless),
//                                 new CANCoder(DriveConstants.kBackLeftAbsoluteEncoderPort),
//                                 DriveConstants.kBackLeftDriveAbsoluteEncoderOffset, SwerveModuleLocation.BackLeft),
//
//        new SwerveModuleConfig<>(new CANSparkMax(DriveConstants.kBackRightDriveMotorPort, MotorType.kBrushless),
//                                 new CANSparkMax(DriveConstants.kBackRightTurningMotorPort, MotorType.kBrushless),
//                                 new CANCoder(DriveConstants.kBackRightAbsoluteEncoderPort),
//                                 DriveConstants.kBackRightDriveAbsoluteEncoderOffset, SwerveModuleLocation.BackRight)
//    };
//
//    SwerveModule<?,?,?>[] modules = SwerveDrive.createModules(ModuleConstants.kDriveMotorGearRatio,
//                                                              ModuleConstants.kTurningMotorGearRatio,
//                                                              Units.inchesToMeters(4), DriveConstants.kWheelBase,
//                                                              DriveConstants.kTrackWidth,
//                                                              DriveConstants.kFreeSpeedRpm,
//                                                              DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
//                                                              DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
//                                                              configs);
//
//    m_drive = new SwerveDrive(modules[0],
//                                modules[1],
//                                modules[2],
//                                modules[3],
//                                new WPI_Pigeon2(DriveConstants.PigeonCANID),
//                                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
//                                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
//                                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
//                                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond,
//                                false);

    m_drive.zeroGyro();
    m_drive.setDeadband(0.5);
    m_drive.setPIDF(0.01, 0, 0, 0, 0, ModuleMotorType.TURNING); // TODO: Change PIDF here.
    m_drive.setPIDF(0.1, 0, 0, 0, 0, ModuleMotorType.DRIVE); // TODO: Change PIDF here.
    m_drive.setAngleDeadband(5);

    SmartDashboard.putData(m_drive);

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
      m_drive.publish(Verbosity.HIGH);
    }
    if (SmartDashboard.getBoolean("Update Swerve Drive", false))
    {
      SmartDashboard.putBoolean("Update Swerve Drive", false);
      m_drive.subscribe();
    }
  }
}
