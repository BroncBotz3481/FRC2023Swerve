package frc.robot.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;

// Copied from team3181's REVSwerve2023 repo
public class SwerveModuleState extends edu.wpi.first.math.kinematics.SwerveModuleState
{

  /**
   * Angular velocity in radians per second. Angular Velocity = omega.
   */
  public double angularVelocityRadPerSecond = 0;

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState()
  {
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle                The angle of the module.
   */
  public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle)
  {
    super(speedMetersPerSecond, angle);
    this.angularVelocityRadPerSecond = 0;
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond        The speed of the wheel of the module.
   * @param angle                       The angle of the module.
   * @param angularVelocityRadPerSecond The angular velocity of the module.
   */
  public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double angularVelocityRadPerSecond)
  {
    super(speedMetersPerSecond, angle);
    this.angularVelocityRadPerSecond = angularVelocityRadPerSecond;
  }
}