package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase
{

  public final  SwerveDrive<CANSparkMax, CANSparkMax> m_drive;
  private final WPI_Pigeon2                           m_gyro = new WPI_Pigeon2(DriveConstants.PigeonCANID);
  //Creates Pigeon2 Gyroscope

  public SwerveSubsystem()
  {
    SwerveModule<CANSparkMax, CANSparkMax, CANCoder> m_frontRight, m_frontLeft, m_backRight, m_backLeft;

    m_frontLeft = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kFrontLeftAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.FrontLeft,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth);

    m_frontRight = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kFrontRightAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.FrontRight,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth);

    m_backLeft = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kBackLeftDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kBackLeftTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kBackLeftAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.BackLeft,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth);

    m_backRight = new SwerveModule<>(
        new CANSparkMax(DriveConstants.kBackRightDriveMotorPort, MotorType.kBrushless),
        new CANSparkMax(DriveConstants.kBackRightTurningMotorPort, MotorType.kBrushless),
        new CANCoder(DriveConstants.kBackRightAbsoluteEncoderPort), SwerveModule.SwerveModuleLocation.BackRight,
        ModuleConstants.kDriveMotorGearRatio, ModuleConstants.kTurningMotorGearRatio,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
        Units.inchesToMeters(4), DriveConstants.kWheelBase,
        DriveConstants.kTrackWidth);

    m_drive = new SwerveDrive<>(m_frontLeft, m_backLeft, m_frontRight, m_backRight, m_gyro,
                                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                                DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                                DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond,
                                false);

    m_drive.zeroGyro();
    m_drive.setDeadband(0.02);
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
    m_drive.synchronizeEncoders();
  }
}
