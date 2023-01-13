package frc.robot.subsystems.swerve;

import static java.util.Objects.requireNonNull;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.kinematics.SwerveModuleState2;
import java.io.Closeable;


/**
 * Swerve module for representing a single swerve module of the robot.
 *
 * @param <DriveMotorType>      Main motor type that drives the wheel.
 * @param <AngleMotorType>      Motor that controls the angle of the wheel.
 * @param <AbsoluteEncoderType> Absolute encoder for the swerve drive module.
 */
public class SwerveModule<DriveMotorType extends MotorController, AngleMotorType extends MotorController,
    AbsoluteEncoderType extends CANCoder>
    implements MotorController, Sendable, AutoCloseable
{

  /**
   * Swerve Module location object relative to the center of the robot.
   */
  public final  Translation2d          swerveModuleLocation;
  /**
   * Motor Controllers for drive motor of the swerve module.
   */
  private final DriveMotorType         m_driveMotor;
  /***
   * Motor Controller for the turning motor of the swerve drive module.
   */
  private final AngleMotorType         m_turningMotor;
  /**
   * Enum representing the swerve module's location on the robot, assuming square.
   */
  private final SwerveModuleLocation   swerveLocation;
  /**
   * Absolute encoder for the swerve module.
   */
  private final AbsoluteEncoderType    absoluteEncoder;
  /**
   * The Distance between centers of right and left wheels in meters.
   */
  private final double                 driveTrainWidth;
  /**
   * Configured sensor range for the Absolute Encoder.
   */
  private final AbsoluteSensorRange    configuredSensorRange   = AbsoluteSensorRange.Unsigned_0_to_360;
  /**
   * Characteristics of the SwerveDriveChasis
   */
  private final double                 wheelDiameter;
  /**
   * The Distance between front and back wheels of the robot in meters.
   */
  private final double                 wheelBase;
  /**
   * The drive gear ratio that is used during configuration of the off-board encoders in the motor controllers.
   */
  public        double                 driveGearRatio          = 1;
  /**
   * Angle offset of the CANCoder at initialization.
   */
  public        double                 angleOffset             = 0;
  /**
   * Maximum speed in meters per second, used to eliminate unnecessary movement of the module.
   */
  public        double                 maxDriveSpeedMPS        = 0;
  /**
   * Inverted drive motor.
   */
  private       boolean                invertedDrive           = false;
  /**
   * Inverted turning motor.
   */
  private       boolean                invertedTurn            = false;
  /**
   * Power to drive motor from -1 to 1.
   */
  private       double                 drivePower              = 0;
  /**
   * Maximum free speed RPM for the steering motor.
   */
  private       double                 maxSteeringFreeSpeedRPM = 0;
  /**
   * kV feed forward for PID
   */
  private       double                 moduleRadkV;
  /**
   * Store the last angle for optimization.
   */
  private       double                 targetAngle             = 0;
  /**
   * Target velocity for the swerve module.
   */
  private       double                 targetVelocity          = 0;
  /**
   * The current angle reported by the absolute encoder.
   */
  private       double                 currentAngle;
  /**
   * Acceptable range between current and desired angle.
   */
  private       double                 angleDeadband           = 5;
  /**
   * Drive feedforward for PID when driving by velocity.
   */
  private final SimpleMotorFeedforward driveFeedforward;
  /**
   * PIDF Values for the modules.
   */
  private       double                 kPdrive, kIdrive, kDdrive, kFdrive, kIZdrive, kPturn, kIturn, kDturn, kFturn,
      kIZturn;
  private SparkMaxPIDController m_drivePIDController;
  private SparkMaxPIDController m_turningPIDController;

  /**
   * Swerve module constructor. Both motors <b>MUST</b> be a {@link MotorController} class. It is recommended to create
   * a command to reset the encoders when triggered and
   *
   * @param mainMotor                 Main drive motor. Must be a {@link MotorController} type.
   * @param angleMotor                Angle motor for controlling the angle of the swerve module.
   * @param encoder                   Absolute encoder for the swerve module.
   * @param driveGearRatio            Drive gear ratio in form of (rotation:1 AKA rotations/1) to get the encoder ticks
   *                                  per rotation.
   * @param steerGearRatio            Steering motor gear ratio (usually 12.8:1 for MK4 in form of rotations:1 or
   *                                  rotations/1), only applied if using Neo's.
   * @param swervePosition            Swerve Module position on the robot.
   * @param steeringOffsetDegrees     The current offset of the absolute encoder from 0 in degrees.
   * @param wheelDiameterMeters       The wheel diameter of the swerve drive module in meters.
   * @param wheelBaseMeters           The Distance between front and back wheels of the robot in meters.
   * @param driveTrainWidthMeters     The Distance between centers of right and left wheels in meters.
   * @param steeringMotorFreeSpeedRPM The RPM free speed of the steering motor.
   * @param maxSpeedMPS               The maximum drive speed in meters per second.
   * @param maxDriveAcceleration      The maximum drive acceleration in meters^2 per second.
   * @throws RuntimeException if an assertion fails or invalid swerve module location is given.
   */
  public SwerveModule(DriveMotorType mainMotor, AngleMotorType angleMotor, AbsoluteEncoderType encoder,
                      SwerveModuleLocation swervePosition, double driveGearRatio, double steerGearRatio,
                      double steeringOffsetDegrees,
                      double wheelDiameterMeters, double wheelBaseMeters, double driveTrainWidthMeters,
                      double steeringMotorFreeSpeedRPM, double maxSpeedMPS, double maxDriveAcceleration)
  {
    requireNonNull(mainMotor);
    requireNonNull(angleMotor);
    requireNonNull(encoder);

    this.wheelDiameter = wheelDiameterMeters;
    this.wheelBase = wheelBaseMeters;
    this.driveTrainWidth = driveTrainWidthMeters;
    maxSteeringFreeSpeedRPM = steeringMotorFreeSpeedRPM;

    // Set the maximum speed for each swerve module for use when trying to optimize movements.
    // Drive feedforward gains
    //        public static final double KS = 0;
    //        public static final double KV = 12 / MAX_SPEED; // Volt-seconds per meter (max voltage divided by max
    //        speed)
    //        public static final double KA = 12 / MAX_ACCELERATION; // Volt-seconds^2 per meter (max voltage divided
    //        by max accel)
    maxDriveSpeedMPS = maxSpeedMPS;
    driveFeedforward = new SimpleMotorFeedforward(0, 12 / maxDriveSpeedMPS, 12 / maxDriveAcceleration);

    m_driveMotor = mainMotor;
    m_turningMotor = angleMotor;
    swerveLocation = swervePosition;

    assert isCTREDriveMotor() || isREVDriveMotor();
    assert isCTRETurningMotor() || isREVTurningMotor();

    this.driveGearRatio = driveGearRatio;
    absoluteEncoder = encoder;
    swerveModuleLocation = getSwerveModulePosition(swervePosition);
    setAngleOffset(steeringOffsetDegrees);
    resetEncoders();

    if (isREVDriveMotor())
    {
      setupREVMotor(((CANSparkMax) mainMotor), SwerveModuleMotorType.DRIVE, driveGearRatio);
      // Might need to sleep for 200ms to 1s before this.
      // burnFlash(SwerveModuleMotorType.DRIVE);
    } else
    {
      setupCTREMotor(((BaseTalon) mainMotor), SwerveModuleMotorType.DRIVE, driveGearRatio);
    }

    if (isREVTurningMotor())
    {
      // MK4 turning motor gear ratio is 12.8:1
      setupREVMotor(((CANSparkMax) angleMotor), SwerveModuleMotorType.TURNING, steerGearRatio);
      // Might need to sleep for 200ms to 1s before this.
      // burnFlash(SwerveModuleMotorType.TURNING);
    } else if (encoder instanceof CANCoder)
    {
      setupCANCoderRemoteSensor(((BaseTalon) angleMotor), encoder);
      setupCTREMotor(((BaseTalon) angleMotor), SwerveModuleMotorType.TURNING, 1);
    }

    encoder.configAbsoluteSensorRange(configuredSensorRange);
    // Convert CANCoder to read data as unsigned 0 to 360 for synchronization purposes.

    // burnFlash(SwerveModuleMotorType.TURNING);
    // burnFlash(SwerveModuleMotorType.DRIVE);
    publish(Verbosity.SETUP);
  }

  /**
   * Convert {@link SwerveModuleLocation} to {@link String} representation.
   *
   * @param swerveLocation Swerve position to convert.
   * @return {@link String} name of the {@link SwerveModuleLocation} enum.
   */
  public static String SwerveModuleLocationToString(SwerveModuleLocation swerveLocation)
  {
    switch (swerveLocation)
    {
      case FrontLeft:
        return "FrontLeft";
      case BackLeft:
        return "BackLeft";
      case FrontRight:
        return "FrontRight";
      case BackRight:
        return "BackRight";
      default:
        return "Unknown";
    }
  }

  /**
   * Configure the magnetic offset in the CANCoder.
   *
   * @param offset Magnetic offset in degrees.
   * @return SwerveModule for one line configuration.
   */
  public SwerveModule setAngleOffset(double offset)
  {
    angleOffset = offset;
    absoluteEncoder.configMagnetOffset(offset);
    return this;
  }

  /**
   * Reset the REV encoders onboard the SparkMax's to 0, and set's the drive motor to position to 0 and synchronizes the
   * internal steering encoders with the absolute encoder.
   */
  public void resetEncoders()
  {
    if (isREVDriveMotor())
    {
      ((CANSparkMax) m_driveMotor).getEncoder().setPosition(0);
    }

    if (isCTREDriveMotor())
    {
      ((BaseTalon) m_driveMotor).setSelectedSensorPosition(0);
    }

    synchronizeSteeringEncoder();

  }

  /**
   * Synchronizes the internal encoder for the steering motor with the value from the absolute encoder.
   */
  public void synchronizeSteeringEncoder()
  {
    if (absoluteEncoder instanceof CANCoder)
    {
      if (absoluteEncoder.getMagnetFieldStrength() != MagnetFieldStrength.Good_GreenLED)
      {
        System.err.println("CANCoder magnetic field strength is unacceptable, will not synchronize encoders.");
        return;
      }
      if (isREVTurningMotor())
      {
        ((CANSparkMax) m_turningMotor).getEncoder().setPosition(absoluteEncoder.getAbsolutePosition());
      }
      if (isCTRETurningMotor())
      {
        ((BaseTalon) m_turningMotor).setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition());
      }
    }
  }

  /**
   * Burn the current settings to flash.
   *
   * @param type Swerve module motor to burn flash of.
   */
  public void burnFlash(SwerveModuleMotorType type)
  {
    if (isREVTurningMotor() || isREVDriveMotor())
    {
      assert (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor) instanceof CANSparkMax;
      ((CANSparkMax) (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).burnFlash();
    }
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   * @param type           Swerve Module Motor to configure.
   * @return Self for one line configuration.
   */
  public SwerveModule setVoltageCompensation(double nominalVoltage, SwerveModuleMotorType type)
  {
    if (isREVDriveMotor() || isREVTurningMotor())
    {
      assert (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor) instanceof CANSparkMax;
      ((CANSparkMax) (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor))
          .enableVoltageCompensation(nominalVoltage);
      // burnFlash(type);
    }
    if (isCTREDriveMotor() || isCTRETurningMotor())
    {
      assert (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor) instanceof BaseTalon;
      ((BaseTalon) (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor))
          .configSetParameter(ParamEnum.eNominalBatteryVoltage, nominalVoltage, 0, 0); // Unsure if this works.

    }
    return this;
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   * @param type         Swerve Drive Motor type to configure.
   * @return Self for one line configuration.
   */
  public SwerveModule setCurrentLimit(int currentLimit, SwerveModuleMotorType type)
  {
    if (isREVTurningMotor() || isREVDriveMotor())
    {
      assert (type == SwerveModuleMotorType.TURNING ? m_turningMotor : m_driveMotor) instanceof CANSparkMax;
      ((CANSparkMax) (type == SwerveModuleMotorType.TURNING ? m_turningMotor : m_driveMotor))
          .setSmartCurrentLimit(currentLimit);
      // burnFlash(type);
    }

    if (isCTREDriveMotor() || isCTRETurningMotor())
    {
      assert (type == SwerveModuleMotorType.TURNING ? m_turningMotor : m_driveMotor) instanceof BaseTalon;
      ((BaseTalon) (type == SwerveModuleMotorType.TURNING ? m_turningMotor : m_driveMotor))
          .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 2), 100);

    }

    return this;
  }


  /**
   * Setup REV motors and configure the values in the class for them. Set's the driveMotorTicksPerRotation, and
   * m_drivePIDController for the class. Assumes the absolute encoder reads from 0 to 360. Configures motor controller
   * in brake mode. Set the smart limit amperage. Setup nominal voltage. Configure the smart limits for drive and
   * steering motors.
   *
   * @param motor                 Motor controller.
   * @param swerveModuleMotorType Turning motor or drive motor.
   * @param gearRatio             Gear ratio for the motor.
   */
  private void setupREVMotor(CANSparkMax motor, SwerveModuleMotorType swerveModuleMotorType, double gearRatio)
  {
    RelativeEncoder encoder = motor.getEncoder();

    motor.restoreFactoryDefaults();
    motor.clearFaults();

    /*motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100); // Applied Output, Faults, Sticky Faults, Is Follower
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
                                 20); // Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Motor Position
    */// TODO: Configure Status Frame 3 and 4 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces

    motor.setIdleMode(IdleMode.kBrake);

    //    motor.setSmartCurrentLimit(40, 60);
    setVoltageCompensation(12, swerveModuleMotorType); // Nominal voltage is 12v

    if (swerveModuleMotorType == SwerveModuleMotorType.DRIVE)
    {
      // Based off current limit used in swerve-lib
      // URL: https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/rev/NeoDriveControllerFactoryBuilder.java#L38

      setCurrentLimit(80, swerveModuleMotorType);
      // motor.getEncoder().getCountsPerRevolution()
      m_drivePIDController = motor.getPIDController();
      m_drivePIDController.setFeedbackDevice(encoder);

      // Based off https://github.com/AusTINCANsProgrammingTeam/2022Swerve/blob/main/2022Swerve/src/main/java/frc/robot/Constants.java
      // Math set's the coefficient to the OUTPUT of the ENCODER (RPM == rot/min) which is the INPUT to the PID.
      // We want to set the PID to use MPS == meters/second :)
      // Dimensional Analysis
      // r/min * K = m/s
      // r/min * 1min/60s * (pi*diameter*gear)/r = m/s
      // r/min * (pi*diameter*gear)/60 = m/s
      // setREVConversionFactor(motor, (Math.PI * wheelDiameter) / (60 * gearRatio), SwerveModuleMotorType.DRIVE);
      // Stolen from https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/SwerveModule.java#L68
      // and https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/Constants.java#L89
      setREVConversionFactor(motor, ((Math.PI * wheelDiameter) / gearRatio) / 60, SwerveModuleMotorType.DRIVE);

    } else
    {
      setCurrentLimit(40, swerveModuleMotorType);
      m_turningPIDController = motor.getPIDController();

      m_turningPIDController.setFeedbackDevice(encoder);
      m_turningPIDController.setPositionPIDWrappingEnabled(true);
      if (configuredSensorRange == AbsoluteSensorRange.Unsigned_0_to_360)
      {
        m_turningPIDController.setPositionPIDWrappingMinInput(0);
        m_turningPIDController.setPositionPIDWrappingMaxInput(360);
      } else
      {
        m_turningPIDController.setPositionPIDWrappingMinInput(-180);
        m_turningPIDController.setPositionPIDWrappingMaxInput(180);
      }
      // Math set's the coefficient to the OUTPUT of the ENCODER (ticks) which is the INPUT to the PID.
      // We want to set the PID to use degrees :)
      // Dimensional Analysis
      // deg * K = ticks
      // deg * (360deg/(42*gearRatio)ticks) = ticks
      // K = 360/(42*gearRatio)
      // setREVConversionFactor(motor, 360 / (42 * gearRatio), SwerveModuleMotorType.TURNING);
      // Sotlen from https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/Constants.java#L91
      // and https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/SwerveModule.java#L53
      setREVConversionFactor(motor, 360 / gearRatio, SwerveModuleMotorType.TURNING);
      moduleRadkV = (12 * 60) / (maxSteeringFreeSpeedRPM * Math.toRadians(360 / gearRatio));

      setPIDF(0.07, 0, 0.3, 0, 10, SwerveModuleMotorType.TURNING);
    }

  }

  /**
   * Get the swerve module position in {@link Translation2d} from the enum passed.
   *
   * @param swerveLocation Swerve module location enum.
   * @return Location as {@link Translation2d}.
   * @throws RuntimeException If Enum value is not defined.
   */
  private Translation2d getSwerveModulePosition(SwerveModuleLocation swerveLocation)
  {
    // Modeling off of https://github.com/Stampede3630/2022-Code/blob/master/src/main/java/frc/robot/SwerveDrive.java
    switch (swerveLocation)
    {
      case FrontLeft:
        return new Translation2d(wheelBase / 2, driveTrainWidth / 2);
      case BackLeft:
        return new Translation2d(-wheelBase / 2, driveTrainWidth / 2);
      case FrontRight:
        return new Translation2d(wheelBase / 2, -driveTrainWidth / 2);
      case BackRight:
        return new Translation2d(-wheelBase / 2, -driveTrainWidth / 2);
      default:
        throw new RuntimeException("Invalid location given");
    }
  }

  /**
   * Set the CANCoder to be the primary PID on the motor controller and configure the PID to accept inputs in degrees.
   * The talon will communicate independently of the roboRIO to fetch the current CANCoder position (which will result
   * in PID adjustments when using a CANivore).
   *
   * @param motor   Talon Motor controller to configure.
   * @param encoder CANCoder to use as the remote sensor.
   */
  private void setupCANCoderRemoteSensor(BaseTalon motor, CANCoder encoder)
  {
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configRemoteFeedbackFilter(encoder, CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
    motor.configSelectedFeedbackCoefficient((double) 360 / 4096); // Degrees/Ticks
    // The CANCoder has 4096 ticks per 1 revolution.
  }

  /**
   * Set up the CTRE motors and configure class attributes correspondingly
   *
   * @param motor                 Motor controller to configure.
   * @param swerveModuleMotorType Motor type to configure
   * @param gearRatio             Gear ratio of the motor for one revolution.
   */
  private void setupCTREMotor(BaseTalon motor, SwerveModuleMotorType swerveModuleMotorType, double gearRatio)
  {

    // Purposely did not configure status frames since CTRE motors should be on a CANivore

    motor.setSensorPhase(true);
    motor.setNeutralMode(NeutralMode.Brake);
    // Unable to use TalonFX configs since this should support both TalonSRX's and TalonFX's
    setVoltageCompensation(12, swerveModuleMotorType);
    // Code is based off of.
    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500SteerControllerFactoryBuilder.java#L91
    // and here
    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L52

    if (swerveModuleMotorType == SwerveModuleMotorType.DRIVE)
    {
      setCurrentLimit(80, swerveModuleMotorType);
      // Math set's the coefficient to the OUTPUT of the ENCODER (ticks/100ms) which is the INPUT to the PID.
      // We want to set the PID to use MPS == meters/second :)
      // Dimensional analysis, solve for K
      // ticks/100ms * K = meters/second
      // ticks/100ms * 100ms/(1s=1000ms) * (pi*diameter)meters/(ticks[4096]*gearRatio)ticks = meters/second
      // ticks/100ms * 1/10 * (pi*diameter)/(ticks[4096]*gearRatio)ticks = meters/second
      // ticks/100ms * (pi*diameter)/((ticks[4096]*gearRatio)*10) = meters/second
      // K = (pi*diameter)/((ticks[4096]*gearRatio)*10)
      // Set the feedback sensor up earlier in setCANRemoteFeedbackSensor()
      motor.configSelectedFeedbackCoefficient(((Math.PI * wheelDiameter) / ((4096 / gearRatio)) * 10));
    } else
    {
      setCurrentLimit(20, swerveModuleMotorType);
      setPIDF(0.2, 0, 0.1, 0, 100, swerveModuleMotorType);
    }
  }

  /**
   * Configures the conversion factor based upon which motor.
   *
   * @param motor                 motor controller to configure
   * @param conversionFactor      Conversion from RPM to MPS for drive motor, and rotations to degrees for the turning
   *                              motor.
   * @param swerveModuleMotorType Turning motor or drive motor for conversion factor setting.
   */
  private void setREVConversionFactor(CANSparkMax motor, double conversionFactor,
                                      SwerveModuleMotorType swerveModuleMotorType)
  {
    if (swerveModuleMotorType == SwerveModuleMotorType.TURNING)
    {
      motor.getEncoder().setPositionConversionFactor(conversionFactor);
      motor.getEncoder().setVelocityConversionFactor(conversionFactor / 60);

    } else
    {
      motor.getEncoder().setVelocityConversionFactor(conversionFactor);
      motor.getEncoder().setPositionConversionFactor(conversionFactor * 60);

    }

  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the SparkMax.
   *
   * @param P                     Proportional gain for closed loop. This is multiplied by closed loop error in sensor
   *                              units. Default is 1.0
   * @param I                     Integral gain for closed loop. This is multiplied by closed loop error in sensor units
   *                              every PID Loop.
   * @param D                     Derivative gain for closed loop. This is multiplied by derivative error (sensor units
   *                              per PID loop). Default is 0.1
   * @param F                     Feed Fwd gain for Closed loop.
   * @param integralZone          Integral Zone can be used to auto clear the integral accumulator if the sensor pos is
   *                              too far from the target. This prevents unstable oscillation if the kI is too large.
   *                              Value is in sensor units.
   * @param swerveModuleMotorType Swerve drive motor type.
   */
  private void setREVPIDF(double P, double I, double D, double F, double integralZone,
                          SwerveModuleMotorType swerveModuleMotorType)
  {
    // Example at
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java#L65-L71
    if (swerveModuleMotorType == SwerveModuleMotorType.DRIVE)
    {
      m_drivePIDController.setP(P, REV_slotIdx.Velocity.ordinal());
      m_drivePIDController.setI(I, REV_slotIdx.Velocity.ordinal());
      m_drivePIDController.setD(D, REV_slotIdx.Velocity.ordinal());
      m_drivePIDController.setFF(F, REV_slotIdx.Velocity.ordinal());
      m_drivePIDController.setIZone(integralZone, REV_slotIdx.Velocity.ordinal());
      m_turningPIDController.setOutputRange(-1, -1, REV_slotIdx.Velocity.ordinal());

    } else
    {
      m_turningPIDController.setP(P, REV_slotIdx.Position.ordinal());
      m_turningPIDController.setI(I, REV_slotIdx.Position.ordinal());
      m_turningPIDController.setD(D, REV_slotIdx.Position.ordinal());
      m_turningPIDController.setFF(F, REV_slotIdx.Position.ordinal());
      m_turningPIDController.setIZone(integralZone, REV_slotIdx.Position.ordinal());
      m_turningPIDController.setOutputRange(-1, -1, REV_slotIdx.Position.ordinal());
    }
    burnFlash(swerveModuleMotorType);
  }


  /**
   * Set the angle using the onboard controller when working with CTRE Talons
   *
   * @param angle Angle in degrees
   */
  private void setCTREAngle(double angle)
  {
    ((BaseTalon) m_turningMotor).set(ControlMode.Position, angle);
    // TODO: Pass feedforward down.
  }

  /**
   * Set the velocity of the drive motor.
   *
   * @param velocity Velocity in meters per second.
   */
  private void setCTREDrive(double velocity)
  {
    ((BaseTalon) m_driveMotor).set(ControlMode.Velocity, driveFeedforward.calculate(velocity));
  }

  /**
   * Set the angle using the onboard controller when working with REV SparkMax's
   *
   * @param angle       angle to set the motor too in degrees.
   * @param feedforward The feedforward for the PID.
   */
  private void setREVAngle(double angle, double feedforward)
  {
    if (feedforward == 0)
    {
      // Intended if setting the angle via crafted unoptimized swerve module state.
      setREVAngle(angle);
    }
    m_turningPIDController.setReference(angle, ControlType.kPosition, REV_slotIdx.Position.ordinal(), feedforward,
                                        ArbFFUnits.kVoltage);
  }

  /**
   * Set the angle using the onboard controller when working with REV SparkMax's
   *
   * @param angle angle to set the motor too in degrees.
   */
  private void setREVAngle(double angle)
  {
    m_turningPIDController.setReference(angle, ControlType.kPosition, REV_slotIdx.Position.ordinal());
  }

  /**
   * Set the REV meters per second for the drive motor.
   *
   * @param velocity Velocity in meters per second.
   */
  private void setREVDrive(double velocity)
  {
    m_drivePIDController.setReference(velocity, ControlType.kVelocity, REV_slotIdx.Velocity.ordinal(),
                                      driveFeedforward.calculate(velocity));
  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the TalonSRX.
   *
   * @param profile               The {@link CTRE_slotIdx} to use.
   * @param P                     Proportional gain for closed loop. This is multiplied by closed loop error in sensor
   *                              units. Note the closed loop output interprets a final value of 1023 as full output. So
   *                              use a gain of '0.25' to get full output if err is 4096u (Mag Encoder 1 rotation)
   * @param I                     Integral gain for closed loop. This is multiplied by closed loop error in sensor units
   *                              every PID Loop. Note the closed loop output interprets a final value of 1023 as full
   *                              output. So use a gain of '0.00025' to get full output if err is 4096u (Mag Encoder 1
   *                              rotation) after 1000 loops
   * @param D                     Derivative gain for closed loop. This is multiplied by derivative error (sensor units
   *                              per PID loop). Note the closed loop output interprets a final value of 1023 as full
   *                              output. So use a gain of '250' to get full output if derr is 4096u per (Mag Encoder 1
   *                              rotation) per 1000 loops (typ 1 sec)
   * @param F                     Feed Fwd gain for Closed loop. See documentation for calculation details. If using
   *                              velocity, motion magic, or motion profile, use (1023 * duty-cycle /
   *                              sensor-velocity-sensor-units-per-100ms)
   * @param integralZone          Integral Zone can be used to auto clear the integral accumulator if the sensor pos is
   *                              too far from the target. This prevents unstable oscillation if the kI is too large.
   *                              Value is in sensor units. (ticks per 100ms)
   * @param swerveModuleMotorType Motor Type for swerve module.
   */
  private void setCTREPIDF(CTRE_slotIdx profile, double P, double I, double D, double F, double integralZone,
                           SwerveModuleMotorType swerveModuleMotorType)
  {
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor
                                                                       : m_turningMotor)).selectProfileSlot(
        profile.ordinal(), CTRE_pidIdx.PRIMARY_PID.ordinal());
    // More Closed-Loop Configs at
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configs-per-slot-four-slots-available
    // Example at
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot/Robot.java
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kP(
        profile.ordinal(), P);
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kI(
        profile.ordinal(), I);
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kD(
        profile.ordinal(), D);
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kF(
        profile.ordinal(), F);

    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor
                                                                       : m_turningMotor)).config_IntegralZone(
        profile.ordinal(), integralZone);

    // If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
    // Value is in sensor units.

    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor
                                                                       : m_turningMotor)).configAllowableClosedloopError(
        profile.ordinal(), 0);

  }


  /**
   * Set the PIDF coefficients for the closed loop PID onboard the motor controller. Tuning the PID
   * <p>
   * <b>P</b> = .5 and increase it by .1 until oscillations occur, then decrease by .05 then .005 until oscillations
   * stop and angle is perfect or near perfect.
   * </p>
   * <p>
   * <b>I</b> = 0 tune this if your PID never quite reaches the target, after tuning <b>D</b>. Increase this by
   * <b>P</b>*.01 each time and adjust accordingly.
   * </p>
   * <p>
   * <b>D</b> = 0 tune this if the PID accelerates too fast, it will smooth the motion. Increase this by <b>P</b>*10
   * and adjust accordingly.
   * </p>
   * <p>
   * <b>F</b> = 0 tune this if the PID is being used for velocity, the <b>F</b> is multiplied by the target and added
   * to the voltage output. Increase this by 0.01 until the PIDF reaches the desired state in a fast enough manner.
   * </p>
   * Documentation for this is best described by CTRE <a
   * href="https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#position-closed-loop-control-mode">here</a>.
   *
   * @param p                     Proportional gain for closed loop. This is multiplied by closed loop error in sensor
   *                              units.
   * @param i                     Integral gain for closed loop. This is multiplied by closed loop error in sensor units
   *                              every PID Loop.
   * @param d                     Derivative gain for closed loop. This is multiplied by derivative error (sensor units
   *                              per PID loop).
   * @param f                     Feed Fwd gain for Closed loop.
   * @param integralZone          Integral Zone can be used to auto clear the integral accumulator if the sensor pos is
   *                              too far from the target. This prevents unstable oscillation if the kI is too large.
   *                              Value is in sensor units.
   * @param swerveModuleMotorType Swerve drive motor type.
   * @return self for one line configuration.
   */
  public SwerveModule setPIDF(double p, double i, double d, double f, double integralZone,
                              SwerveModuleMotorType swerveModuleMotorType)
  {
    if (swerveModuleMotorType == SwerveModuleMotorType.TURNING)
    {
      kPturn = p;
      kIturn = i;
      kDturn = d;
      kFturn = f;
      kIZturn = integralZone;
    } else
    {
      kPdrive = p;
      kIdrive = i;
      kDdrive = d;
      kFdrive = f;
      kIZdrive = integralZone;
    }

    if (isREVTurningMotor() || isREVDriveMotor())
    {
      assert (swerveModuleMotorType == SwerveModuleMotorType.TURNING ? m_turningMotor
                                                                     : m_driveMotor) instanceof CANSparkMax;
      setREVPIDF(p, i, d, f, integralZone, swerveModuleMotorType);
    }
    if (isCTREDriveMotor() || isCTRETurningMotor())
    {
      assert (swerveModuleMotorType == SwerveModuleMotorType.TURNING ? m_turningMotor
                                                                     : m_driveMotor) instanceof BaseTalon;
      setCTREPIDF(swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? CTRE_slotIdx.Velocity : CTRE_slotIdx.Distance,
                  p, i, d, f, integralZone, swerveModuleMotorType);
    }

    return this;
  }

  /**
   * Initializes this {@link Sendable} object.
   *
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType(SwerveModuleLocationToString(swerveLocation) + " SwerveDriveModule");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
//    if (isCTREDriveMotor())
//    {
//      builder.addDoubleProperty("Drive Motor Velocity MPS", ((BaseTalon) m_driveMotor)::getSelectedSensorVelocity,
//                                this::setCTREDrive);
//
//    } else
//    {
//      builder.addDoubleProperty("Drive Motor Velocity MPS", ((CANSparkMax) m_driveMotor).getEncoder()::getVelocity,
//                                this::setREVDrive);
//    }
//    if (isCTRETurningMotor())
//    {
//      builder.addDoubleProperty("Steering Motor Angle Degrees", ((BaseTalon) m_turningMotor)
//      ::getSelectedSensorPosition,
//                                this::setCTREAngle);
//    } else
//    {
//      builder.addDoubleProperty("Steering Motor Angle Degrees",
//                                ((CANSparkMax) m_turningMotor).getEncoder()::getPosition,
//                                this::setREVAngle);
//    }
//    if (absoluteEncoder instanceof CANCoder)
//    {
//      builder.addBooleanProperty("CANCoder Magnet",
//                                 () -> absoluteEncoder.getMagnetFieldStrength() == MagnetFieldStrength.Good_GreenLED,
//                                 null);
//    }
  }

  /**
   * Set the angle deadband for the setAngle function.
   *
   * @param deadband Deadband angle in degrees.
   */
  public void setAngleDeadband(double deadband)
  {
    angleDeadband = deadband;
  }

  /**
   * Set the angle of the swerve module.
   *
   * @param angle       Angle in degrees.
   * @param feedforward The feedforward for the PID.
   */
  private void setAngle(double angle, double feedforward)
  {
    targetAngle = angle;

    angle += 180; // Since the angle is given in the form of -180 to 180, we add 180 to make it 0 to 360.
    assert angle <= 360;
    // currentAngle is always updated in getState which is called during setState which calls this function.
    if ((angle - angleDeadband) <= currentAngle && currentAngle <= (angle + angleDeadband))
    {
      m_turningMotor.setVoltage(0);
      return;
    }

    if (isREVTurningMotor())
    {
      setREVAngle(angle, feedforward);
    } else
    {
      setCTREAngle(angle);
    }
  }

  /**
   * Set the angle of the swerve module without feedforward.
   *
   * @param angle Angle in degrees.
   */
  private void setAngle(double angle)
  {
    angle += 180; // Since the angle is given in the form of -180 to 180, we add 180 to make it 0 to 360.
    assert angle <= 360;

    if (isREVTurningMotor())
    {
      setREVAngle(angle);
    } else
    {
      setCTREAngle(angle);
    }
  }


  /**
   * Set the drive motor velocity in MPS.
   *
   * @param velocity Velocity in MPS.
   */
  public void setVelocity(double velocity)
  {
    targetVelocity = velocity;
    if (isCTREDriveMotor())
    {
      setCTREDrive(velocity);
    } else
    {
      setREVDrive(velocity);
    }
  }

  /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  @Override
  public void set(double speed)
  {
    drivePower = speed;
    m_driveMotor.set(speed);
  }

  /**
   * Common interface for getting the current set speed of a motor controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  @Override
  public double get()
  {
    return drivePower;
  }

  /**
   * Returns whether the turning motor is a CTRE motor.
   *
   * @return is the turning motor a CTRE motor?
   */
  private boolean isCTRETurningMotor()
  {
    return m_turningMotor instanceof BaseMotorController;
  }

  /**
   * Returns whether the drive motor is a CTRE motor. All CTRE motors implement the {@link BaseMotorController} class.
   * We will only support the TalonSRX and TalonFX.
   *
   * @return is the drive motor a CTRE motor?
   */
  private boolean isCTREDriveMotor()
  {
    return m_driveMotor instanceof TalonFX || m_driveMotor instanceof TalonSRX;
  }

  /**
   * Returns whether the drive motor is a REV motor. The only REV Motor Controller is the SparkMax. We will not support
   * {@link PWMSparkMax}
   *
   * @return is the drive motor a SparkMax?
   */
  private boolean isREVTurningMotor()
  {
    return m_turningMotor instanceof CANSparkMax;
  }

  /**
   * Returns whether the drive motor is a CTRE motor.
   *
   * @return is the drive motor a SparkMax?
   */
  private boolean isREVDriveMotor()
  {
    return m_driveMotor instanceof CANSparkMax;
  }

  /**
   * Common interface for returning if a motor controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  @Override
  public boolean getInverted()
  {
    return invertedDrive;
  }

  /**
   * Common interface for inverting direction of a motor controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    invertedDrive = isInverted;
    m_driveMotor.setInverted(isInverted);
  }

  /**
   * Set the steering motor to be inverted.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInvertedSteering(boolean isInverted)
  {
    invertedTurn = isInverted;
    m_turningMotor.setInverted(isInverted);
  }

  /**
   * Set the sensor to be inverted for the motor type.
   *
   * @param isInverted     The state of inversion, true is inverted.
   * @param invertAbsolute Invert the absolute encoder if the type is SwerveModuleMotorType.TURNING too.
   * @param type           Swerve module motor's sensors to configure.
   */
  public void setInvertedSensor(boolean isInverted, SwerveModuleMotorType type, boolean invertAbsolute)
  {
    if (isREVTurningMotor() || isREVDriveMotor())
    {
      assert (type == SwerveModuleMotorType.TURNING ? m_turningMotor : m_driveMotor) instanceof CANSparkMax;
      ((CANSparkMax) (type == SwerveModuleMotorType.TURNING ? m_turningMotor : m_driveMotor)).getEncoder().setInverted(
          invertedDrive);
    }
    // TODO: Implement CTRE inversion.

    if (type == SwerveModuleMotorType.TURNING && invertAbsolute)
    {
      if (absoluteEncoder instanceof CANCoder)
      {
        absoluteEncoder.configSensorDirection(isInverted);
      }
    }
  }

  /**
   * Disable the motor controller.
   */
  @Override
  public void disable()
  {
    stopMotor();
  }

  /**
   * Check that the link is good on the swerve module and CAN bus is able to retrieve data.
   *
   * @return true on all devices are accessible over CAN.
   */
  public boolean activeCAN()
  {
    // Based off of https://github.com/DigitalDislocators/SDS-MK4i-NEO-Swerve-Template/blob/main/src/main/java/frc/robot/subsystems/SwerveModuleSparkMax.java#L490
    boolean drive = true, turn = true, encoder = true;
    if (isREVDriveMotor())
    {
      drive = ((CANSparkMax) m_driveMotor).getFirmwareVersion() != 0;
    }
    if (isREVTurningMotor())
    {
      turn = ((CANSparkMax) m_turningMotor).getFirmwareVersion() != 0;
    }
    if (absoluteEncoder instanceof CANCoder)
    {
      encoder = absoluteEncoder.getFirmwareVersion() > 0;
    }
    // TODO: Implement CTRE
    return drive && turn && encoder;

  }

  /**
   * Get the module state.
   *
   * @param range Sensor range to retrieve angle in, will convert if different from configured.
   * @return SwerveModuleState with the encoder inputs.
   * @throws RuntimeException Exception if CANCoder doesnt exist
   */
  public SwerveModuleState2 getState(AbsoluteSensorRange range)
  {
    double     mps = 0;
    Rotation2d angle;
    if (absoluteEncoder instanceof CANCoder)
    {
      currentAngle = absoluteEncoder.getAbsolutePosition();
      if (range != configuredSensorRange)
      {
        currentAngle += (configuredSensorRange == AbsoluteSensorRange.Unsigned_0_to_360 &&
                         range == AbsoluteSensorRange.Signed_PlusMinus180) ? -180 : 180;
      }
      angle = Rotation2d.fromDegrees(currentAngle);
    } else
    {
      throw new RuntimeException("No CANCoder attached.");
    }
    if (isCTREDriveMotor())
    {
      mps = (((BaseTalon) m_driveMotor).getSelectedSensorVelocity());
    } else
    {
      mps = (((CANSparkMax) m_driveMotor).getEncoder().getVelocity());
    }
    return new SwerveModuleState2(mps, angle, 0);
  }

  /**
   * Get the module state.
   *
   * @return SwerveModuleState with the encoder inputs.
   * @throws RuntimeException Exception if CANCoder doesnt exist
   */
  public SwerveModuleState2 getState()
  {
    return getState(configuredSensorRange);
  }

  /**
   * Get the swerve module position based off the sensors.
   *
   * @return Swerve Module position, with the angle as what the angle is supposed to be, not what it actually is.
   */
  public SwerveModulePosition getPosition()
  {
    double distanceMeters = isREVDriveMotor() ? ((CANSparkMax) m_driveMotor).getEncoder().getPosition()
                                              : ((BaseTalon) m_driveMotor).getSelectedSensorPosition();

    return new SwerveModulePosition(distanceMeters, Rotation2d.fromDegrees(targetAngle));
    ///^ Assume our current angle is what it is supposed to be.
  }

  /**
   * Set the module speed and angle based off the module state.
   *
   * @param state Module state.
   */
  public void setState(SwerveModuleState2 state)
  {
    // inspired by https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/SwerveModule.java#L22
    state = new SwerveModuleState2(SwerveModuleState2.optimize(state,
                                                               getState(
                                                                   AbsoluteSensorRange.Signed_PlusMinus180).angle));
    /*
    double angle = (Math.abs(state.speedMetersPerSecond) <= (maxDriveSpeedMPS * 0.01) ?
                    lastAngle :
                    state.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
    */ // Commented out since we want to test rotations.
    double angle = state.angle.getDegrees();

    setAngle(angle, state.angularVelocityRadPerSecond * moduleRadkV);
    setVelocity(state.speedMetersPerSecond);

  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the motor.
   */
  @Override
  public void stopMotor()
  {
//    m_driveMotor.stopMotor();
//    m_turningMotor.stopMotor();
    m_turningMotor.set(0);
    m_driveMotor.set(0);
  }

  /**
   * Closes this resource, relinquishing any underlying resources. This method is invoked automatically on objects
   * managed by the {@code try}-with-resources statement.
   *
   * <p>While this interface method is declared to throw {@code
   * Exception}, implementers are <em>strongly</em> encouraged to declare concrete implementations of the {@code close}
   * method to throw more specific exceptions, or to throw no exception at all if the close operation cannot fail.
   *
   * <p> Cases where the close operation may fail require careful
   * attention by implementers. It is strongly advised to relinquish the underlying resources and to internally
   * <em>mark</em> the resource as closed, prior to throwing the exception. The {@code close} method is unlikely to be
   * invoked more than once and so this ensures that the resources are released in a timely manner. Furthermore it
   * reduces problems that could arise when the resource wraps, or is wrapped, by another resource.
   *
   * <p><em>Implementers of this interface are also strongly advised
   * to not have the {@code close} method throw {@link InterruptedException}.</em>
   * <p>
   * This exception interacts with a thread's interrupted status, and runtime misbehavior is likely to occur if an
   * {@code InterruptedException} is {@linkplain Throwable#addSuppressed suppressed}.
   * <p>
   * More generally, if it would cause problems for an exception to be suppressed, the {@code AutoCloseable.close}
   * method should not throw it.
   *
   * <p>Note that unlike the {@link Closeable#close close}
   * method of {@link Closeable}, this {@code close} method is <em>not</em> required to be idempotent.  In other words,
   * calling this {@code close} method more than once may have some visible side effect, unlike {@code Closeable.close}
   * which is required to have no effect if called more than once.
   * <p>
   * However, implementers of this interface are strongly encouraged to make their {@code close} methods idempotent.
   *
   * @throws Exception if this resource cannot be closed
   */
  @Override
  public void close() throws Exception
  {
    SendableRegistry.remove(this);
  }

  /**
   * Subscribe from data within smart dashboard to make changes to the swerve module.
   */
  public void subscribe()
  {
    String name = "SwerveDrive/" + SwerveModule.SwerveModuleLocationToString(swerveLocation);

    // PID
    double velocity = SmartDashboard.getNumber(name + "/drive/pid/setpoint", targetVelocity);
    kPdrive = SmartDashboard.getNumber(name + "/drive/pid/kP", kPdrive);
    kIdrive = SmartDashboard.getNumber(name + "/drive/pid/kI", kIdrive);
    kDdrive = SmartDashboard.getNumber(name + "/drive/pid/kD", kDdrive);
    kFdrive = SmartDashboard.getNumber(name + "/drive/pid/kF", kFdrive);
    kIZdrive = SmartDashboard.getNumber(name + "/drive/pid/kIZ", kIZdrive);
    setPIDF(kPdrive, kIdrive, kDdrive, kFdrive, kIZdrive, SwerveModuleMotorType.DRIVE);
    if (velocity != targetVelocity)
    {
      setVelocity(velocity);
    }

    double angle = SmartDashboard.getNumber(name + "/steer/pid/setpoint", targetAngle);
    kPturn = SmartDashboard.getNumber(name + "/steer/pid/kP", kPturn);
    kIturn = SmartDashboard.getNumber(name + "/steer/pid/kI", kIturn);
    kDturn = SmartDashboard.getNumber(name + "/steer/pid/kD", kDturn);
    kFturn = SmartDashboard.getNumber(name + "/steer/pid/kF", kFturn);
    kIZturn = SmartDashboard.getNumber(name + "/steer/pid/kIZ", kIZturn);
    setPIDF(kPturn, kIturn, kDturn, kFturn, kIZturn, SwerveModuleMotorType.TURNING);
    if (angle != targetAngle)
    {
      setAngle(angle);
    }

    // Deadband
    double deadband = SmartDashboard.getNumber(name + "/steer/pid/deadband", angleDeadband);
    if (deadband != angleDeadband)
    {
      setAngleDeadband(deadband);
    }

    // Offset
    double offset = SmartDashboard.getNumber(name + "/steer/encoder/offset", angleOffset);
    if (angleOffset != offset)
    {
      setAngleOffset(offset);
      synchronizeSteeringEncoder();
    }

    // Inversion
    boolean turnInvert = SmartDashboard.getBoolean(name + "/steer/inverted", invertedTurn);
    if (turnInvert != invertedTurn)
    {
      setInvertedSteering(turnInvert);
    }

    boolean driveInvert = SmartDashboard.getBoolean(name + "/drive/inverted", invertedDrive);
    if (driveInvert != invertedDrive)
    {
      setInverted(driveInvert);
    }

  }

  /**
   * Publish data to the smart dashboard relating to this swerve moduule.
   *
   * @param level Verbosity level, affects the CAN utilization, on HIGH it will enable the update button.
   */
  public void publish(Verbosity level)
  {
    String name = "SwerveDrive/" + SwerveModule.SwerveModuleLocationToString(swerveLocation); // TODO: Move to attribute
    switch (level)
    {
      case SETUP:
        // PID
        SmartDashboard.putNumber(name + "/drive/pid/kP", kPdrive);
        SmartDashboard.putNumber(name + "/drive/pid/kI", kIdrive);
        SmartDashboard.putNumber(name + "/drive/pid/kD", kDdrive);
        SmartDashboard.putNumber(name + "/drive/pid/kF", kFdrive);
        SmartDashboard.putNumber(name + "/drive/pid/kIZ", kIZdrive);

        SmartDashboard.putNumber(name + "/steer/pid/kP", kPturn);
        SmartDashboard.putNumber(name + "/steer/pid/kI", kIturn);
        SmartDashboard.putNumber(name + "/steer/pid/kD", kDturn);
        SmartDashboard.putNumber(name + "/steer/pid/kF", kFturn);
        SmartDashboard.putNumber(name + "/steer/pid/kIZ", kIZturn);
        SmartDashboard.putNumber(name + "/steer/pid/deadband", angleDeadband);

        // Inverted Motors.
        SmartDashboard.putBoolean(name + "/steer/inverted", invertedTurn);
        SmartDashboard.putBoolean(name + "/drive/inverted", invertedDrive);

        // Angle Constants
        SmartDashboard.putNumber(name + "/steer/encoder/offset", angleOffset);

      case HIGH:
        // Update if button is set.
        if (SmartDashboard.getBoolean(name + "/update", false))
        {
          SmartDashboard.putBoolean(name + "/update", false);
          subscribe();
        }

        // The higher the better, 2 and 3 are what we want.
        SmartDashboard.putNumber(name + "/steer/encoder/field", absoluteEncoder.getMagnetFieldStrength().value);
      case NORMAL:
        double integratedPosition = isREVTurningMotor() ? ((CANSparkMax) m_turningMotor).getEncoder().getPosition() :
                                    absoluteEncoder.getAbsolutePosition();
        double integratedVelocity = isREVDriveMotor() ? ((CANSparkMax) m_driveMotor).getEncoder().getVelocity() : 0;
        // TODO: Implement for CTRE
        // Steering Encoder Values
        SmartDashboard.putNumber(name + "/steer/encoder/integrated", integratedPosition);
        SmartDashboard.putNumber(name + "/steer/encoder/absolute", absoluteEncoder.getAbsolutePosition());

        // Driving Encoder Values
        SmartDashboard.putNumber(name + "/drive/encoder/velocity", integratedVelocity);

        // CAN Bus is accessible
        SmartDashboard.putBoolean(name + "/status", activeCAN());
      case LOW:
        // PID
        SmartDashboard.putNumber(name + "/drive/pid/target", targetVelocity);
        SmartDashboard.putNumber(name + "/steer/pid/target", targetAngle);

    }
  }

  /**
   * Motor type for the swerve drive module
   */
  public enum SwerveModuleMotorType
  {
    /**
     * Drive Motor
     */
    DRIVE,
    /**
     * Steering Motor
     */
    TURNING
  }

  /**
   * The Talon SRX Slot profile used to configure the motor to use for the PID.
   */
  enum CTRE_slotIdx
  {
    Distance, Turning, Velocity, MotionProfile
  }

  /**
   * REV Slots for PID configuration.
   */
  enum REV_slotIdx
  {
    Position, Velocity, Simulation
  }

  /**
   * The TalonSRX PID to use onboard.
   */
  enum CTRE_pidIdx
  {
    PRIMARY_PID, AUXILIARY_PID, THIRD_PID, FOURTH_PID
  }

  enum CTRE_remoteSensor
  {
    REMOTE_SENSOR_0, REMOTE_SENSOR_1
  }

  /**
   * Swerve Module location on the robot.
   */
  public enum SwerveModuleLocation
  {
    /**
     * Swerve Module for the front left of the robot chassis.
     */
    FrontLeft,
    /**
     * Swerve Module for the back left of the robot chassis.
     */
    BackLeft,
    /**
     * Swerve Module for the front right of the robot chassis.
     */
    FrontRight,
    /**
     * Swerve Module for the back right of the robot chassis.
     */
    BackRight
  }

  /**
   * Verbosity levels for data publishing,
   */
  public enum Verbosity
  {
    /**
     * The bare minimum and not utilize the CAN bus when reporting data. Only posts data from attributes.
     */
    LOW,
    /**
     * Utilize the CAN bus minimally.
     */
    NORMAL,
    /**
     * Extensively use the CAN bus to fetch data and report back.
     */
    HIGH,
    /**
     * Creates every field for the module.
     */
    SETUP
  }
}