package frc.robot.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Copied from team3181's REVSwerve2023 repo
public class SwerveModuleState2 extends edu.wpi.first.math.kinematics.SwerveModuleState
{

  /**
   * Angular velocity in radians per second. Angular Velocity = omega.
   */
  public double angularVelocityRadPerSecond = 0;

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState2()
  {
  }

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState2(SwerveModuleState self)
  {
    super(self.speedMetersPerSecond, self.angle);
    this.angularVelocityRadPerSecond = 0;
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle                The angle of the module.
   */
  public SwerveModuleState2(double speedMetersPerSecond, Rotation2d angle)
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
  public SwerveModuleState2(double speedMetersPerSecond, Rotation2d angle, double angularVelocityRadPerSecond)
  {
    super(speedMetersPerSecond, angle);
    this.angularVelocityRadPerSecond = angularVelocityRadPerSecond;
  }
}