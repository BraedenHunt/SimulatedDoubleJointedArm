// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/* Abstract class to drive trajectories with a swerve drive robot */
public class DriveTrajectory extends CommandBase {
    protected final DrivetrainSubsystem swerveSubsystem;
    protected final Timer timer;

    private final HolonomicDriveController controller;
    protected Trajectory trajectory;

    /** Creates a new DriveHolonomicTrajectory. */
    public DriveTrajectory(final DrivetrainSubsystem swerveSubsystem, Trajectory trajectory) {
        this.swerveSubsystem = swerveSubsystem;
        this.trajectory = trajectory;

        this.timer = new Timer();
        controller = new HolonomicDriveController(
                new PIDController(0, 0, 0), // x
                                            // controller
                new PIDController(0, 0, 0), // y
                                            // controller
                new ProfiledPIDController(0, 0, 0, // Theta
                        // controller
                        new TrapezoidProfile.Constraints(180, 180))); // Max angular
                                                                      // acceleration
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveSubsystem.setCurrentRobotPose(trajectory.getInitialPose());
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var goal = trajectory.sample(timer.get());
        var adjustedSpeeds = controller.calculate(swerveSubsystem.getCurrentRobotPose(), goal, getHeading(goal));
        swerveSubsystem.drive(adjustedSpeeds);

        SmartDashboard.putNumber("Robot Goal Position X", goal.poseMeters.getX());
        SmartDashboard.putNumber("Robot Goal Position Y", goal.poseMeters.getY());
        SmartDashboard.putNumber("Robot Goal Position Theta", goal.poseMeters.getRotation().getRadians());
    }

    // Override this method to implement custom path following.
    public Rotation2d getHeading(State goalState) {
        return new Rotation2d();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerveSubsystem.stop();
        System.out.println("Ending driving");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

}