package frc.robot.subsystems.swerve;

public class CTRESwerveMotor extends SwerveMotor
{

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the SparkMax.
   *
   * @param P            Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
   *                     Default is 1.0
   * @param I            Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
   *                     PID Loop.
   * @param D            Derivative gain for closed loop. This is multiplied by derivative error (sensor units per PID
   *                     loop). Default is 0.1
   * @param F            Feed Fwd gain for Closed loop.
   * @param integralZone Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too far
   *                     from the target. This prevents unstable oscillation if the kI is too large. Value is in sensor
   *                     units.
   */
  @Override
  public void setPIDF(double P, double I, double D, double F, double integralZone)
  {

  }

  /**
   * Configures the conversion factor based upon which motor.
   *
   * @param conversionFactor Conversion from RPM to MPS for drive motor, and rotations to degrees for the turning
   *                         motor.
   */
  @Override
  public void setConversionFactor(double conversionFactor)
  {

  }

  /**
   * Set the target for the PID loop.
   *
   * @param target      The PID target to aim for.
   * @param feedforward The feedforward for this target.
   */
  @Override
  public void setTarget(double target, double feedforward)
  {

  }

  /**
   * Stop the motor by sending 0 volts to it.
   */
  @Override
  public void stop()
  {

  }

  /**
   * Set the speed of the drive motor from -1 to 1.
   *
   * @param speed Speed from -1 to 1.
   */
  @Override
  public void set(double speed)
  {

  }

  /**
   * Get the current value of the encoder corresponding to the PID.
   *
   * @return Current value of the encoder.
   */
  @Override
  public double getCurrent()
  {
    return 0;
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {

  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {

  }

  /**
   * Set the encoder value.
   *
   * @param value Value to set the encoder to.
   */
  @Override
  public void setEnocder(double value)
  {

  }

  /**
   * Check that the link is good on the swerve module and CAN bus is able to retrieve data.
   *
   * @return true on all devices are accessible over CAN.
   */
  @Override
  public boolean reachable()
  {
    return false;
  }

  /**
   * Optimize the CAN status frames to reduce utilization.
   */
  @Override
  public void optimizeCANFrames()
  {

  }

  /**
   * Save configuration data to the motor controller so it is persistent on reboot.
   */
  @Override
  public void saveConfig()
  {

  }

  /**
   * Invert the motor.
   *
   * @param inverted Set the motor as inverted.
   */
  @Override
  public void setInverted(boolean inverted)
  {

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

//  /**
//   * Set the PIDF coefficients for the closed loop PID onboard the TalonSRX.
//   *
//   * @param profile         The {@link CTRE_slotIdx} to use.
//   * @param P               Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
//   *                        Note the closed loop output interprets a final value of 1023 as full output. So use a gain
//   *                        of '0.25' to get full output if err is 4096u (Mag Encoder 1 rotation)
//   * @param I               Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
//   *                        PID Loop. Note the closed loop output interprets a final value of 1023 as full output. So
//   *                        use a gain of '0.00025' to get full output if err is 4096u (Mag Encoder 1 rotation) after
//   *                        1000 loops
//   * @param D               Derivative gain for closed loop. This is multiplied by derivative error (sensor units per
//   *                        PID loop). Note the closed loop output interprets a final value of 1023 as full output. So
//   *                        use a gain of '250' to get full output if derr is 4096u per (Mag Encoder 1 rotation) per
//   *                        1000 loops (typ 1 sec)
//   * @param F               Feed Fwd gain for Closed loop. See documentation for calculation details. If using velocity,
//   *                        motion magic, or motion profile, use (1023 * duty-cycle /
//   *                        sensor-velocity-sensor-units-per-100ms)
//   * @param integralZone    Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too
//   *                        far from the target. This prevents unstable oscillation if the kI is too large. Value is in
//   *                        sensor units. (ticks per 100ms)
//   * @param moduleMotorType Motor Type for swerve module.
//   */
//  private void setCTREPIDF(CTRE_slotIdx profile, double P, double I, double D, double F, double integralZone,
//                           ModuleMotorType moduleMotorType)
//  {
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor
//                                                           : m_turningMotor)).selectProfileSlot(
//        profile.ordinal(), CTRE_pidIdx.PRIMARY_PID.ordinal());
//    // More Closed-Loop Configs at
//    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configs-per-slot-four-slots-available
//    // Example at
//    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot/Robot.java
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kP(
//        profile.ordinal(), P);
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kI(
//        profile.ordinal(), I);
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kD(
//        profile.ordinal(), D);
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kF(
//        profile.ordinal(), F);
//
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor
//                                                           : m_turningMotor)).config_IntegralZone(
//        profile.ordinal(), integralZone);
//
//    // If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
//    // Value is in sensor units.
//
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor
//                                                           : m_turningMotor)).configAllowableClosedloopError(
//        profile.ordinal(), 0);
//
//  }
//
//  /**
//   * Set the angle using the onboard controller when working with CTRE Talons
//   *
//   * @param angle Angle in degrees
//   */
//  private void setCTREAngle(double angle)
//  {
//    ((BaseTalon) m_turningMotor).set(ControlMode.Position, angle);
//    // TODO: Pass feedforward down.
//  }
//
//  /**
//   * Set the velocity of the drive motor.
//   *
//   * @param velocity Velocity in meters per second.
//   */
//  private void setCTREDrive(double velocity)
//  {
//    ((BaseTalon) m_driveMotor).set(ControlMode.Velocity, driveFeedforward.calculate(velocity));
//  }
//
//  /**
//   * Set up the CTRE motors and configure class attributes correspondingly
//   *
//   * @param motor           Motor controller to configure.
//   * @param moduleMotorType Motor type to configure
//   * @param gearRatio       Gear ratio of the motor for one revolution.
//   */
//  private void setupCTREMotor(BaseTalon motor, ModuleMotorType moduleMotorType, double gearRatio)
//  {
//
//    // Purposely did not configure status frames since CTRE motors should be on a CANivore
//
//    motor.setSensorPhase(true);
//    motor.setNeutralMode(NeutralMode.Brake);
//    // Unable to use TalonFX configs since this should support both TalonSRX's and TalonFX's
//    setVoltageCompensation(12, moduleMotorType);
//    // Code is based off of.
//    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500SteerControllerFactoryBuilder.java#L91
//    // and here
//    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L52
//
//    if (moduleMotorType == ModuleMotorType.DRIVE)
//    {
//      setCurrentLimit(80, moduleMotorType);
//      // Math set's the coefficient to the OUTPUT of the ENCODER (ticks/100ms) which is the INPUT to the PID.
//      // We want to set the PID to use MPS == meters/second :)
//      // Dimensional analysis, solve for K
//      // ticks/100ms * K = meters/second
//      // ticks/100ms * 100ms/(1s=1000ms) * (pi*diameter)meters/(ticks[4096]*gearRatio)ticks = meters/second
//      // ticks/100ms * 1/10 * (pi*diameter)/(ticks[4096]*gearRatio)ticks = meters/second
//      // ticks/100ms * (pi*diameter)/((ticks[4096]*gearRatio)*10) = meters/second
//      // K = (pi*diameter)/((ticks[4096]*gearRatio)*10)
//      // Set the feedback sensor up earlier in setCANRemoteFeedbackSensor()
//      motor.configSelectedFeedbackCoefficient(((Math.PI * wheelDiameter) / ((4096 / gearRatio)) * 10));
//    } else
//    {
//      setCurrentLimit(20, moduleMotorType);
//      setPIDF(0.2, 0, 0.1, 0, 100, moduleMotorType);
//    }
//  }
//
//  /**
//   * Set the CANCoder to be the primary PID on the motor controller and configure the PID to accept inputs in degrees.
//   * The talon will communicate independently of the roboRIO to fetch the current CANCoder position (which will result
//   * in PID adjustments when using a CANivore).
//   *
//   * @param motor   Talon Motor controller to configure.
//   * @param encoder CANCoder to use as the remote sensor.
//   */
//  private void setupCTRECANCoderRemoteSensor(BaseTalon motor, CANCoder encoder)
//  {
//    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
//    motor.configRemoteFeedbackFilter(encoder, CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
//    motor.configSelectedFeedbackCoefficient((double) 360 / 4096); // Degrees/Ticks
//    // The CANCoder has 4096 ticks per 1 revolution.
//  }
//
//  /**
//   * Returns whether the turning motor is a CTRE motor.
//   *
//   * @return is the turning motor a CTRE motor?
//   */
//  private boolean isCTRETurningMotor()
//  {
//    return m_turningMotor instanceof BaseMotorController;
//  }
//
//  /**
//   * Returns whether the drive motor is a CTRE motor. All CTRE motors implement the {@link BaseMotorController} class.
//   * We will only support the TalonSRX and TalonFX.
//   *
//   * @return is the drive motor a CTRE motor?
//   */
//  private boolean isCTREDriveMotor()
//  {
//    return m_driveMotor instanceof TalonFX || m_driveMotor instanceof TalonSRX;
//  }

}
