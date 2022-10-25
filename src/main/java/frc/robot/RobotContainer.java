// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick        driverJoytick   = new Joystick(Constants.OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Renamed JoystickSwerveCmd to SwerveDriveCommand
    swerveSubsystem.setDefaultCommand(
        new SwerveDriveCommand(swerveSubsystem, () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                               () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                               () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                               () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    // Configure the button bindings
//    controller0 = new XboxController(0);
//    controller1 = new XboxController(1);
    configureButtonBindings();
  }

  private void configureButtonBindings()
  {
    new JoystickButton(driverJoytick, 2).whenPressed(swerveSubsystem::zeroHeading); // NEW
  }

  public Command getAutonomousCommand()
  {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                             AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(
        DriveConstants.kDriveKinematics);

    // 2. Generate trajectory ---Is all the Poses put together in Autonomous
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                   List.of(new Translation2d(1, 0),
                                                                           new Translation2d(1, -1)),
                                                                   new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                                                                   trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                                                      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    // Changed to use the wpilib SwerveControllerCommand
    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.html
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose,
                                                                                  DriveConstants.kDriveKinematics,
                                                                                  xController, yController,
                                                                                  thetaController,
                                                                                  swerveSubsystem::setModuleStates,
                                                                                  swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), //Import?
        swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}