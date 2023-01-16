// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.DriveTrajectory;

public class DrivetrainSubsystem extends SubsystemBase {

  private SwerveDrivePoseEstimator m_poseEstimator;
  private SwerveDriveKinematics m_kinematics;

  private ChassisSpeeds m_chassisSpeeds;

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {
    /*
     * m_kinematics = new
     * SwerveDriveKinematics(SwerveDriveConstants.kSwerveModuleLocations);
     * 
     * m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
     * 
     * m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, new
     * Rotation2d(),
     * new SwerveModulePosition[] { new SwerveModulePosition(), new
     * SwerveModulePosition(),
     * new SwerveModulePosition(), new SwerveModulePosition() },
     * new Pose2d());
     */
  }

  @Override
  public void periodic() {
    // SwerveModuleState[] states =
    // m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states,
    // SwerveDriveConstants.kMaxVelocity);

    /*
     * for (int i = 0; i < m_swerveModules.length; i++) {
     * m_swerveModules[i].set(
     * states[i].speedMetersPerSecond / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND *
     * Drivetrain.MAX_VOLTAGE,
     * states[i].angle.getRadians());
     * }
     */
    // var gyroAngle = this.getGyroscopeRotation();

    // Update the pose
    // m_poseEstimator.update(gyroAngle, null);
  }

  public void zeroGyroscope() {
    // System.out.println("Zeroing Gyroscope");
    // m_navx.zeroYaw();
    // Reset the odometry with new 0 heading but same position.
    // m_odometry.resetPosition(m_odometry.getPoseMeters(), new Rotation2d());
  }

  /**
   * Calibrates the gyroscope. This should only be called on robotinit because
   * it takes some time to run.
   */
  public void calibrateGyroscope() {
    // m_navx.calibrate();
  }

  public Rotation2d getGyroscopeRotation() {
    // // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return new Rotation2d();// Rotation2d.fromDegrees(-m_navx.getYaw());
  }

  public Pose2d getCurrentRobotPose() {
    return new Pose2d();// m_poseEstimator.getEstimatedPosition();
  }

  public void setCurrentRobotPose(Pose2d pose) {
    // m_poseEstimator.resetPosition(getGyroscopeRotation(), null, pose);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    // m_chassisSpeeds = chassisSpeeds;
  }

  public void stop() {

  }

  public Command getDriveTrajectoryCommand(Trajectory traj) {
    return new DriveTrajectory(this, traj);
  }

  private Field2d m_field;

  public void simulationInit() {
    m_field = new Field2d();
    m_field.setRobotPose(new Pose2d());
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void simulationPeriodic() {
    Pose2d goalPose = new Pose2d(SmartDashboard.getNumber("Robot Goal Position X", 0),
                                 SmartDashboard.getNumber("Robot Goal Position Y", 0),
                                 new Rotation2d(
                                    SmartDashboard.getNumber("Robot Goal Position X", 0)));
    m_field.setRobotPose(goalPose);
  }
}
