package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule
{

  private final CANSparkMax driveMotor; //Two Neo Motors on each module with Spark Max Motor Controllers
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder; //Accesses build in Encoder to the DriveMotor
  private final RelativeEncoder turningEncoder; //Accesses build in Encoder to the TurningMotor

  private final PIDController turningPidController; //Need to use the built-in PID controller on the Spark-Max's to
  // control the Angle

  private final CANCoder absoluteEncoder; //Connected to the Turning Motor (Is an "Absolute" Encoder because as the
  // robot turns off and power is lost,
  // the robot always knows the permanent location the wheels are facing
  private final boolean  absoluteEncoderReversed; //Variable to store whether the Absolute Encoder is "Reversed" or not
  private final double   absoluteEncoderOffsetRad; //Variable to store the Absolute Encoder's Offset Position from 0

  /**
   *
   * @param driveMotorId
   * @param turningMotorId
   * @param driveMotorReversed
   * @param turningMotorReversed
   * @param absoluteEncoderId
   * @param absoluteEncoderOffset
   * @param absoluteEncoderReversed
   */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                      //Used to create swerve modules using their Motor ID's
                      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
  {           //and if they are reversed or have an Offset value

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset; //Initializes
    this.absoluteEncoderReversed = absoluteEncoderReversed; //Initializes
    absoluteEncoder = new CANCoder(absoluteEncoderId); //Creates CANCoder Encoder

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless); //Creates DriveMotors
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless); //Creates TurningMotors

    driveMotor.setInverted(driveMotorReversed); //Reverses Driver Motor
    turningMotor.setInverted(turningMotorReversed); //Reverses Turning Motor

    driveEncoder = driveMotor.getEncoder(); //Gets the Drive Motor's Encoders
    turningEncoder = turningMotor.getEncoder(); //Gets the Turning Motor's Encoders

    //Sets Conversion Constants to work in Meters and Radians instead of rotations (Math Located in Constants.java)
    driveEncoder.setPositionConversionFactor(
        ModuleConstants.kDriveEncoderRot2Meter); //Converts Driver Motor from Rotations to Meters
    driveEncoder.setVelocityConversionFactor(
        ModuleConstants.kDriveEncoderRPM2MeterPerSec); //Converts Drive Motor from Revolutions Per Min to Meters Per
    // Second
    turningEncoder.setPositionConversionFactor(
        ModuleConstants.kTurningEncoderRot2Rad); //Converts Turning Motor Rotations to Radians
    turningEncoder.setVelocityConversionFactor(
        ModuleConstants.kTurningEncoderRPM2RadPerSec); //Converts Turning Motor Revolutions Per Min to Radians Per Sec

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0); //Initializes the Turning PID Controller

    turningPidController.enableContinuousInput(-Math.PI,
                                               Math.PI); //Connects -180 degrees and 180 degrees, essentially telling
    // the computer that We have Circle

    resetEncoders(); //Resets Encoders everytime robot is powered on
  }

  //Methods to get the Encoder Values
  //Built in Encoder-
  public double getDrivePosition()
  {
    return driveEncoder.getPosition();
  }

  //Built in Encoder-

  /**
   * Position of built in encoder on the turning motor.
   *
   * @return Rotation in radians.
   */
  public double getTurningPosition()
  {
    return turningEncoder.getPosition();
  }

  //Built in Encoder-
  public double getDriveVelocity()
  {
    return driveEncoder.getVelocity();
  }

  //Built in Encoder-
  public double getTurningVelocity()
  {
    return driveEncoder.getVelocity();
  }

  /**
   * CANCoder
   *
   * @return Encoder position in radians
   */
  public double getAbsoluteEncoderRad()
  {
    return Rotation2d.fromDegrees(absoluteEncoder.getPosition()).getRadians() *
           (absoluteEncoderReversed ? -1.0 : 1.0); //Multiples by -1 if Encoder is Reversed
  }

  //Gives Built in Encoders Absolute Encoder Information
  public void resetEncoders()
  {
    driveEncoder.setPosition(0); //Resets Drive Motor to Zero
    turningEncoder.setPosition(getAbsoluteEncoderRad()); //Turning Encoder is set to Absolute Encoder Value
  }

  //Method to return Swerve Module State
  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  //Actuates this Module
  public void setDesiredState(SwerveModuleState state)
  {
    //Stops motors and exits the function if driving velocity is close to 0
//    if (Math.abs(state.speedMetersPerSecond) < 0.001)
//    {
//      stop();
//      return;
//    }
    state = SwerveModuleState.optimize(state,
                                       getState().angle); //Optimizes so that you will never have to move more than
    // 90 degrees at a time
    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, attainableMaxSpeedMetersPerSecond);
    driveMotor.set(state.speedMetersPerSecond /
                   Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //Scales Velocity down using Max Speed

    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
  }

  public void stop()
  {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}

