// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.AdaptiveTrajectoryGeneration;
import frc.robot.utilities.AdaptiveTrajectoryGeneration.Bay;
import frc.robot.utilities.AdaptiveTrajectoryGeneration.Position;

public class AdaptiveDriveCommand extends DriveTrajectory {
  /** Creates a new AdaptiveDriveCommand. */
  private Bay m_bay;
  private Position m_position;

  public AdaptiveDriveCommand(DrivetrainSubsystem swerveSubsystem, Bay bay, Position position) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(swerveSubsystem, null);
    this.m_bay = bay;
    this.m_position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.trajectory = AdaptiveTrajectoryGeneration.generateTrajectory(swerveSubsystem.getCurrentRobotPose(), m_bay, m_position);
    super.initialize();
  }

}
