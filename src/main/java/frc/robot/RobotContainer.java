// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ArmWristSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmWristSubsystem m_armWristSubsystem = new ArmWristSubsystem(true);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

      // 2018 cross scale auto waypoints.
      var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
      Rotation2d.fromDegrees(-180));
      var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
      Rotation2d.fromDegrees(-160));

      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
      config.setReversed(true);

      var trajectory = TrajectoryGenerator.generateTrajectory(
        sideStart,
        interiorWaypoints,
        crossScale,
        config);
      
      return m_armWristSubsystem.getGotoCommand(0, 180)  .andThen(
  m_drivetrainSubsystem.getDriveTrajectoryCommand(trajectory)).andThen(m_armWristSubsystem.getGotoCommand(130, 0));

  }

  private SendableChooser presetCommands = new SendableChooser<Command>();

  public void simulationInit() {

    System.out.println("Working Directory = " + System.getProperty("user.dir"));
    m_armWristSubsystem.useBrakeAssist(true);
    m_armWristSubsystem.simulationInit();

    m_drivetrainSubsystem.simulationInit();

    presetCommands.addOption("High Cone", m_armWristSubsystem.getGotoCommand(130, 0));

    presetCommands.addOption("Hybrid Cone", m_armWristSubsystem.getGotoCommand(100, 20));
    presetCommands.addOption("Hybrid Cube", m_armWristSubsystem.getGotoCommand(100, 160));

    presetCommands.setDefaultOption("Substation Intake Cone", m_armWristSubsystem.getGotoCommand(55, 0));
    presetCommands.addOption("Substation Intake Cube", m_armWristSubsystem.getGotoCommand(55, 0));

    SmartDashboard.putData(presetCommands);

    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
        Rotation2d.fromDegrees(-180));
    var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
        Rotation2d.fromDegrees(-160));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(true);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        sideStart,
        interiorWaypoints,
        crossScale,
        config);
    SmartDashboard.putData("Drive trajectory", (Sendable) m_drivetrainSubsystem.getDriveTrajectoryCommand(trajectory));

  }

  public void simulationPeriodic() {
    SmartDashboard.putData("Run Preset Command", (Sendable) presetCommands.getSelected());
  }
}
