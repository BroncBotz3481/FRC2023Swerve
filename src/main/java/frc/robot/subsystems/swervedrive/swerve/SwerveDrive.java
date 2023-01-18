package frc.robot.subsystems.swervedrive.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.swerve.SwerveModule.SwerveModuleLocation;
import frc.robot.subsystems.swervedrive.swerve.SwerveModule.Verbosity;
import frc.robot.subsystems.swervedrive.swerve.SwerveMotor.ModuleMotorType;
import frc.robot.subsystems.swervedrive.swerve.kinematics.SwerveDriveKinematics2;
import frc.robot.subsystems.swervedrive.swerve.kinematics.SwerveModuleState2;
import java.io.Closeable;

/**
 * SwerveDrive base which is meant to be platform agnostic. This implementation expect second order kinematics because
 * second order kinematics prevents the drift that builds up when using first order kinematics. As per this <a
 * href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">ChiefDelphi
 * post</a>
 */
public class SwerveDrive extends RobotDriveBase implements Sendable, AutoCloseable
{


  /**
   * Count of SwerveModule instances created.
   */
  private static int instances;

  /**
   * Front left swerve drive
   */
  private final SwerveModule<?, ?, ?>    m_frontLeft;
  /**
   * Back left swerve drive
   */
  private final SwerveModule<?, ?, ?>    m_backLeft;
  /**
   * Front right swerve drive
   */
  private final SwerveModule<?, ?, ?>    m_frontRight;
  /**
   * Back right swerve drive
   */
  private final SwerveModule<?, ?, ?>    m_backRight;
  /**
   * Swerve drive kinematics.
   */
  private final SwerveDriveKinematics2   m_swerveKinematics;
  /**
   * Swerve drive pose estimator for attempting to figure out our current position.
   */
  private final SwerveDrivePoseEstimator m_swervePoseEstimator;
  /**
   * Pigeon 2.0 centered on the robot.
   */
  private final WPI_Pigeon2              m_pigeonIMU;
  /**
   * Field2d displayed on shuffleboard with current position.
   */
  private final Field2d                  m_field = new Field2d();
  /**
   * The slew rate limiters to make control smooth.
   */
  private final SlewRateLimiter          m_xLimiter, m_yLimiter, m_turningLimiter;
  /**
   * Maximum speed in meters per second.
   */
  public double m_maxSpeedMPS = 5, m_maxAngularVelocity;
  /**
   * Invert the gyro reading.
   */
  private boolean m_gyroInverted = false;


  /**
   * Constructor for Swerve Drive assuming modules have been created and configured with PIDF and conversions.
   *
   * @param frontLeft                              Front left swerve module configured.
   * @param backLeft                               Back left swerve module.
   * @param frontRight                             Front right swerve module.
   * @param backRight                              Back right swerve moduule
   * @param pigeon                                 Pigeon IMU.
   * @param maxSpeedMetersPerSecond                Maximum speed for all modules to follow.
   * @param maxAngularVelocityRadiansPerSecond     Maximum angular velocity for turning when using the drive function.
   * @param maxDriveAccelerationMetersPerSecond    Maximum acceleration in meters per second for the drive motors.
   * @param maxAngularAccelerationRadiansPerSecond Maximum angular acceleration in meters per second for the steering
   *                                               motors.
   * @param gyroInverted                           Invert the gyroscope for the robot.
   */
  public SwerveDrive(SwerveModule<?, ?, ?> frontLeft,
                     SwerveModule<?, ?, ?> frontRight,
                     SwerveModule<?, ?, ?> backLeft,
                     SwerveModule<?, ?, ?> backRight, WPI_Pigeon2 pigeon,
                     double maxSpeedMetersPerSecond, double maxAngularVelocityRadiansPerSecond,
                     double maxDriveAccelerationMetersPerSecond, double maxAngularAccelerationRadiansPerSecond,
                     boolean gyroInverted)
  {
    instances++;
    m_frontLeft = frontLeft;
    m_backRight = backRight;
    m_backLeft = backLeft;
    m_frontRight = frontRight;
    m_gyroInverted = gyroInverted;
    m_swerveKinematics = new SwerveDriveKinematics2(frontLeft.swerveModuleLocation,
                                                    frontRight.swerveModuleLocation,
                                                    backLeft.swerveModuleLocation,
                                                    backRight.swerveModuleLocation);

    m_maxSpeedMPS = maxSpeedMetersPerSecond;
    m_maxAngularVelocity = maxAngularVelocityRadiansPerSecond;
    m_pigeonIMU = pigeon;

    configurePigeonIMU(); // Reset pigeon to 0 and default settings.
    m_swervePoseEstimator = new SwerveDrivePoseEstimator(
        m_swerveKinematics,
        getRotation(),
        getPositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1), // x,y,heading in radians; state std dev, higher=less weight
        VecBuilder.fill(0.9, 1.0, 0.9)); // x,y,heading in radians; Vision measurement std dev, higher=less weight

    m_xLimiter = new SlewRateLimiter(maxDriveAccelerationMetersPerSecond);
    m_yLimiter = new SlewRateLimiter(maxDriveAccelerationMetersPerSecond);
    m_turningLimiter = new SlewRateLimiter(maxAngularAccelerationRadiansPerSecond);

    // Inspired by https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/subsystems/Swerve.java
    SmartDashboard.putData(m_field);
    SmartDashboard.putData(m_pigeonIMU);
  }

  /**
   * Create swerve drive modules
   *
   * @param driveGearRatio            Drive gear ratio in form of (rotation:1 AKA rotations/1) to get the encoder ticks
   *                                  per rotation.
   * @param steerGearRatio            Steering motor gear ratio (usually 12.8:1 for MK4 in form of rotations:1 or
   *                                  rotations/1), only applied if using Neo's.
   * @param wheelDiameterMeters       The wheel diameter of the swerve drive module in meters.
   * @param wheelBaseMeters           The Distance between front and back wheels of the robot in meters.
   * @param driveTrainWidthMeters     The Distance between centers of right and left wheels in meters.
   * @param steeringMotorFreeSpeedRPM The RPM free speed of the steering motor.
   * @param maxSpeedMPS               The maximum drive speed in meters per second.
   * @param maxDriveAcceleration      The maximum drive acceleration in meters^2 per second.
   * @param configs                   The swerve module configuration classes for the swerve drive given.
   * @return Array of swerve modules in the order of front left, front right, back left, back right.
   */
  public static SwerveModule<?, ?, ?>[] createModules(
      double driveGearRatio, double steerGearRatio, double wheelDiameterMeters, double wheelBaseMeters,
      double driveTrainWidthMeters, double steeringMotorFreeSpeedRPM, double maxSpeedMPS,
      double maxDriveAcceleration, SwerveModuleConfig<?, ?, ?>[] configs)
  {
    SwerveModule<?, ?, ?>[] modules = new SwerveModule[configs.length];
    for (int i = 0; i < configs.length; i++)
    {
      int loc;
      switch (configs[i].loc)
      {
        case FrontLeft:
          loc = 0;
          break;
        case BackLeft:
          loc = 2;
          break;
        case FrontRight:
          loc = 1;
          break;
        case BackRight:
          loc = 3;
          break;
        default:
          loc = i;
      }

      modules[loc] = configs[i].createModule(driveGearRatio, steerGearRatio, wheelDiameterMeters, wheelBaseMeters,
                                             driveTrainWidthMeters, steeringMotorFreeSpeedRPM, maxSpeedMPS,
                                             maxDriveAcceleration);
    }
    return modules;
  }

  /**
   * Configure the PigeonIMU with factory default settings and a zeroed gyroscope.
   */
  public void configurePigeonIMU()
  {
    m_pigeonIMU.configFactoryDefault();
    zeroGyro();
  }

  /**
   * Update the swerve drive odometry.
   *
   * @return Swerve drive odometry.
   */
  public SwerveDrivePoseEstimator update()
  {
    m_swervePoseEstimator.update(getRotation(), getPositions());
    return m_swervePoseEstimator;
  }

  /**
   * Drive swerve based off controller input.
   *
   * @param forward       The speed (-1 to 1) in which the Y axis should go.
   * @param strafe        The speed (-1 to 1) in which the X axis should go.
   * @param turn          The speed (-1 to 1) in which the robot should turn.
   * @param fieldRelative Whether or not to use field relative controls.
   */
  public void drive(double forward, double strafe, double turn, boolean fieldRelative)
  {
    // 2. Apply deadband/Dead-Zone
    forward = Math.abs(forward) > m_deadband ? forward : 0.0;
    strafe = Math.abs(strafe) > m_deadband ? strafe : 0.0;
    turn = Math.abs(turn) > m_deadband ? turn : 0.0;

    // 3. Make the driving smoother
    forward = m_xLimiter.calculate(forward) * m_maxSpeedMPS;
    strafe = m_yLimiter.calculate(strafe) * m_maxSpeedMPS;
    turn = m_turningLimiter.calculate(turn) * m_maxAngularVelocity;

    set(forward, strafe, turn, fieldRelative);
  }

  /**
   * Swerve drive function
   *
   * @param forward         forward meters per second, strafe facing left from alliance wall
   * @param strafe          strafe meters per second, forward away from alliance wall
   * @param radianPerSecond radians per second
   * @param fieldRelative   field relative
   */
  public void set(double forward, double strafe, double radianPerSecond, boolean fieldRelative)
  {

    SwerveModuleState2[] moduleStates = m_swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, radianPerSecond, getRotation())
                      : new ChassisSpeeds(forward, strafe, radianPerSecond));
    // try
    // {
    setModuleStates(moduleStates);
    // } catch (Exception e)
    // {
    // System.err.println("Cannot set swerve module states!");
    // }

    try
    {
      this.update();
      m_field.setRobotPose(m_swervePoseEstimator.getEstimatedPosition());
    } catch (Exception e)
    {
      System.err.println("Cannot update SwerveDrive Odometry!");
    }

  }

  /**
   * Set the swerve module states given an array of states. Normalize the wheel speeds to abide by maximum supplied
   *
   * @param states Module states in a specified order. [front left, front right, back left, back right]
   * @throws RuntimeException If the CANCoder is inaccessible or not configured.
   */
  public void setModuleStates(SwerveModuleState2[] states)
  {
    feedWatchdog(); // Required
    SwerveDriveKinematics2.desaturateWheelSpeeds(states, m_maxSpeedMPS);
    m_frontLeft.setState(states[0]);
    m_frontRight.setState(states[1]);
    m_backLeft.setState(states[2]);
    m_backRight.setState(states[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX()
  {
    setModuleStates(new SwerveModuleState2[]{new SwerveModuleState2(0, Rotation2d.fromDegrees(45), 0),
                                             new SwerveModuleState2(0, Rotation2d.fromDegrees(-45), 0),
                                             new SwerveModuleState2(0, Rotation2d.fromDegrees(-45), 0),
                                             new SwerveModuleState2(0, Rotation2d.fromDegrees(45), 0)});
  }

  /**
   * Invert the gyroscope reading.
   *
   * @param isInverted Inversion of the gryoscope, true is inverted.
   */
  public void setGyroInverted(boolean isInverted)
  {
    m_gyroInverted = isInverted;
  }

  /**
   * Get the current robot rotation.
   *
   * @return {@link Rotation2d} of the robot.
   */
  public Rotation2d getRotation()
  {
    return m_gyroInverted ? Rotation2d.fromDegrees(360 - m_pigeonIMU.getYaw()) : Rotation2d.fromDegrees(
        m_pigeonIMU.getYaw());
  }

  /**
   * Get the current Pose, used in autonomous.
   *
   * @return Current pose based off odometry.
   */
  public Pose2d getPose()
  {
    return m_swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Get current swerve module positions in order.
   *
   * @return Swerve module positions array.
   */
  public SwerveModulePosition[] getPositions()
  {
    return new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                      m_backLeft.getPosition(), m_backRight.getPosition()};
  }

  /**
   * Reset the odometry given the position and using current rotation from the PigeonIMU 2.
   *
   * @param pose Current position on the field.
   */
  public void resetOdometry(Pose2d pose)
  {
    m_swervePoseEstimator.resetPosition(getRotation(), getPositions(), pose);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders()
  {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /**
   * Set the current rotation of the gyroscope (pigeonIMU 2) to 0.
   */
  public void zeroGyro()
  {
    m_pigeonIMU.setYaw(0);
  }

  /**
   * Stop all running and turning motors.
   */
  @Override
  public void stopMotor()
  {
    m_frontRight.stopMotor();
    m_backLeft.stopMotor();
    m_frontLeft.stopMotor();
    m_backRight.stopMotor();
  }

  /**
   * Update swerve module parameters based on data in the dashboard.
   */
  public void subscribe()
  {
    m_frontRight.subscribe();
    m_frontLeft.subscribe();
    m_backLeft.subscribe();
    m_backRight.subscribe();
  }

  /**
   * Publish data from the Swerve Modules to the dashboard.
   *
   * @param level Verbosity level in terms of CAN utilization.
   */
  public void publish(Verbosity level)
  {
    m_frontRight.publish(level);
    m_frontLeft.publish(level);
    m_backRight.publish(level);
    m_backLeft.publish(level);
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
   * @param p               Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
   * @param i               Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
   *                        PID Loop.
   * @param d               Derivative gain for closed loop. This is multiplied by derivative error (sensor units per
   *                        PID loop).
   * @param f               Feed Fwd gain for Closed loop.
   * @param integralZone    Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too
   *                        far from the target. This prevents unstable oscillation if the kI is too large. Value is in
   *                        sensor units.
   * @param moduleMotorType Swerve drive motor type.
   */
  public void setPIDF(double p, double i, double d, double f, double integralZone,
                      ModuleMotorType moduleMotorType)
  {
    m_frontRight.setPIDF(p, i, d, f, integralZone, moduleMotorType);
    m_frontLeft.setPIDF(p, i, d, f, integralZone, moduleMotorType);
    m_backRight.setPIDF(p, i, d, f, integralZone, moduleMotorType);
    m_backLeft.setPIDF(p, i, d, f, integralZone, moduleMotorType);
  }

  /**
   * Set the angle deadband for swerve modules.
   *
   * @param deadband Deadband angle in degrees.
   */
  public void setAngleDeadband(double deadband)
  {
    m_backRight.setAngleDeadband(deadband);
    m_backLeft.setAngleDeadband(deadband);
    m_frontLeft.setAngleDeadband(deadband);
    m_frontRight.setAngleDeadband(deadband);
  }

  /**
   * Synchronize internal steering encoders with the absolute encoder.
   */
  public void synchronizeEncoders()
  {
    m_backRight.synchronizeSteeringEncoder();
    m_backLeft.synchronizeSteeringEncoder();
    m_frontRight.synchronizeSteeringEncoder();
    m_frontLeft.synchronizeSteeringEncoder();
  }

  /**
   * Get the description of the robot drive base.
   *
   * @return string of the RobotDriveBase
   */
  @Override
  public String getDescription()
  {
    return "Swerve Drive Base";
  }

  /**
   * Initializes this {@link Sendable} object.
   *
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder)
  {
//    builder.setSmartDashboardType("SwerveDrive");
//    SendableRegistry.addChild(this, m_frontLeft);
//    SendableRegistry.addChild(this, m_frontRight);
//    SendableRegistry.addChild(this, m_backLeft);
//    SendableRegistry.addChild(this, m_backRight);
//    SendableRegistry.addChild(this, m_field);
//    SendableRegistry.addChild(this, m_pigeonIMU);
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
   * Helper class for easier swerve module creation
   *
   * @param <DriveMotorType>    The motor type for the drive motor on the swerve moduule.
   * @param <SteeringMotorType> The motor type for the steering motor on the module.
   * @param <AbsoluteEncoder>   The absolute encoder type.
   */
  public static class SwerveModuleConfig<DriveMotorType extends MotorController,
      SteeringMotorType extends MotorController,
      AbsoluteEncoder extends CANCoder>
  {

    public DriveMotorType       drive;
    public SteeringMotorType    steering;
    public AbsoluteEncoder      encoder;
    public double               angleOffset;
    public SwerveModuleLocation loc;

    /**
     * Swerve Module configuration class to define the motor CAN IDs and absolute encoder offset of a swerve module.
     *
     * @param driveMotor      Drive motor for the swerve module.
     * @param steerMotor      Steer motor for the swerve module.
     * @param encoderSteering CANCoder for the steering motor on the swerve module.
     * @param offset          Absolute encoder offset.
     * @param location        Swerve Moduule location on the chassis.
     */
    public SwerveModuleConfig(DriveMotorType driveMotor, SteeringMotorType steerMotor, AbsoluteEncoder encoderSteering,
                              double offset,
                              SwerveModuleLocation location)
    {
      drive = driveMotor;
      steering = steerMotor;
      angleOffset = offset;
      encoder = encoderSteering;
      loc = location;
    }

    /**
     * Create the swerve module from the configuration.
     *
     * @param driveGearRatio            Drive gear ratio in form of (rotation:1 AKA rotations/1) to get the encoder
     *                                  ticks per rotation.
     * @param steerGearRatio            Steering motor gear ratio (usually 12.8:1 for MK4 in form of rotations:1 or
     *                                  rotations/1), only applied if using Neo's.
     * @param wheelDiameterMeters       The wheel diameter of the swerve drive module in meters.
     * @param wheelBaseMeters           The Distance between front and back wheels of the robot in meters.
     * @param driveTrainWidthMeters     The Distance between centers of right and left wheels in meters.
     * @param steeringMotorFreeSpeedRPM The RPM free speed of the steering motor.
     * @param maxSpeedMPS               The maximum drive speed in meters per second.
     * @param maxDriveAcceleration      The maximum drive acceleration in meters^2 per second.
     * @return The Swerve Module.
     */
    public SwerveModule<DriveMotorType, SteeringMotorType, AbsoluteEncoder> createModule(double driveGearRatio,
                                                                                         double steerGearRatio,
                                                                                         double wheelDiameterMeters,
                                                                                         double wheelBaseMeters,
                                                                                         double driveTrainWidthMeters,
                                                                                         double steeringMotorFreeSpeedRPM,
                                                                                         double maxSpeedMPS,
                                                                                         double maxDriveAcceleration)
    {
      return new SwerveModule<>(drive, steering, encoder, loc,
                                driveGearRatio, steerGearRatio,
                                angleOffset, wheelDiameterMeters,
                                wheelBaseMeters,
                                driveTrainWidthMeters,
                                steeringMotorFreeSpeedRPM,
                                maxSpeedMPS, maxDriveAcceleration);
    }
  }

}