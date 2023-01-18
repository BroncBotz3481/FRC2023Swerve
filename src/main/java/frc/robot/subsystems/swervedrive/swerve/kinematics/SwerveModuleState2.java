package frc.robot.subsystems.swervedrive.swerve.kinematics;

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

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially reversing the direction
   * the wheel spins. Customized from WPILib's version to include placing in appropriate scope for CTRE and REV onboard
   * control as both controllers as of writing don't have support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState2 optimize(
      SwerveModuleState2 desiredState, Rotation2d currentAngle)
  {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta       = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90)
    {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState2(targetSpeed, Rotation2d.fromDegrees(-targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle       Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle)
  {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0)
    {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else
    {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound)
    {
      newAngle += 360;
    }
    while (newAngle > upperBound)
    {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180)
    {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180)
    {
      newAngle += 360;
    }
    return newAngle;
  }
}