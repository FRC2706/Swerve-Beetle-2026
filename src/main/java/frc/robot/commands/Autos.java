// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Utility factory for autonomous commands.
 *
 * NOTE: This file contains a working example auto and a safe stub for a PathPlanner + YAGSL-based
 * auto. Replace the stub implementation in pathPlannerAuto(...) with your project's concrete
 * PathPlanner / YAGSL code once those libraries and your swerve APIs are available in the project.
 */
public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /**
   * Create an autonomous command that follows a PathPlanner path using YAGSL.
   *
   * This method currently provides a compile-safe stub (resets heading and waits).
   * Replace the body with your actual PathPlanner + YAGSL follow command. Example (pseudocode):
   *
   * // PSEUDOCODE (not compiled) - replace after adding PathPlanner/YAGSL dependencies:
   * // PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, maxVel, maxAccel);
   * // PPSwerveControllerCommand follow =
   * //    new PPSwerveControllerCommand(
   * //        traj,
   * //        swerve::getPose,
   * //        swerve.getKinematics(),
   * //        new PIDController(...), new PIDController(...),
   * //        new ProfiledPIDController(...),
   * //        swerve::setModuleStates,
   * //        true);
   * // return Commands.sequence(
   * //     Commands.runOnce(() -> swerve.resetOdometry(traj.getInitialHolonomicPose())),
   * //     follow);
   */
  public static Command pathPlannerAuto(SwerveSubsystem swerve, String pathName) {
    // Safe default that compiles: reset gyro/odometry and wait briefly.
    return Commands.sequence(
        Commands.runOnce(() -> {
          // Reset to a known starting state; replace with actual path initial pose reset
          swerve.zeroGyro();
          // If your SwerveSubsystem exposes resetOdometry(Pose2d), call it here with the path's initial pose.
          // e.g. swerve.resetOdometry(traj.getInitialHolonomicPose());
        }),
        Commands.waitSeconds(0.1) // placeholder - replace with the actual path-follow command
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
