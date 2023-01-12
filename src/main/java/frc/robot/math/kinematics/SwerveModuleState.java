package frc.robot.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;

// Copied from team3181's REVSwerve2023 repo
public class SwerveModuleState extends edu.wpi.first.math.kinematics.SwerveModuleState
{

  public double omegaRadPerSecond = 0;

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
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle                The angle of the module.
   * @param omegaRadPerSecond    The angular velocity of the module.
   */
  public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double omegaRadPerSecond)
  {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
    this.omegaRadPerSecond = omegaRadPerSecond;
  }
}