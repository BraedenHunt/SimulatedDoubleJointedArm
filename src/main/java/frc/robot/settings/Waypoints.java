// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class Waypoints {

    public static final Pose2d kLeftBayLeftConePose = new Pose2d();
    public static final Pose2d kLeftBayCubePose = new Pose2d();
    public static final Pose2d kLeftBayRightConePose = new Pose2d();

    public static final Pose2d kCenterBayLeftConePose = new Pose2d();
    public static final Pose2d kCenterBayCubePose = new Pose2d(2.35, 3.0, new Rotation2d());
    public static final Pose2d kCenterBayRightConePose = new Pose2d();

    public static final Pose2d kRightBayLeftConePose = new Pose2d();
    public static final Pose2d kRightBayCubePose = new Pose2d();
    public static final Pose2d kRightBayRightConePose = new Pose2d();
}
