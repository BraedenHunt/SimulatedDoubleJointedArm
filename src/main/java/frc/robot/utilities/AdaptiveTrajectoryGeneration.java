// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import static java.util.Map.entry;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.settings.Waypoints;

/** Add your docs here. */
public class AdaptiveTrajectoryGeneration {
    public static enum Position {
        ConeLeft, ConeRight, Cube
    }

    public static enum Bay {
        Left, Center, Right
    }

    private static Map<Bay, Map<Position, Pose2d>> locationPoseMap;

    private AdaptiveTrajectoryGeneration() {}

    private static Map<Bay, Map<Position, Pose2d>> getLocationPoseMap() {
        
        var bayMap = new HashMap<Bay, Map<Position,Pose2d>>();

        var leftBayMap = new HashMap<Position, Pose2d>();
        leftBayMap.put(Position.ConeLeft, Waypoints.kLeftBayLeftConePose);
        leftBayMap.put(Position.Cube, Waypoints.kLeftBayCubePose);
        leftBayMap.put(Position.ConeRight, Waypoints.kLeftBayRightConePose);

        var centerBayMap = new HashMap<Position, Pose2d>();
        centerBayMap.put(Position.ConeLeft, Waypoints.kCenterBayLeftConePose);
        centerBayMap.put(Position.Cube, Waypoints.kCenterBayCubePose);
        centerBayMap.put(Position.ConeRight, Waypoints.kCenterBayRightConePose);

        var rightBayMap = new HashMap<Position, Pose2d>();
        rightBayMap.put(Position.ConeLeft, Waypoints.kRightBayLeftConePose);
        rightBayMap.put(Position.Cube, Waypoints.kRightBayCubePose);
        rightBayMap.put(Position.ConeRight, Waypoints.kRightBayRightConePose);


        bayMap.put(Bay.Left, leftBayMap);
        bayMap.put(Bay.Center, centerBayMap);
        bayMap.put(Bay.Right, rightBayMap);

        return bayMap;

    }

    public static Trajectory generateTrajectory(Pose2d initialPose, Bay bay, Position pos) {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(true);
        if (locationPoseMap == null) {
            locationPoseMap = getLocationPoseMap();
        }

        Pose2d endingPose = locationPoseMap.get(bay).get(pos);

        List<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

        if (Math.abs(endingPose.getX() - initialPose.getX()) > 3.5) { 
            // Robot is on other side of charging station, must go around
            interiorWaypoints.add(new Translation2d(5.25, 1.25));
            interiorWaypoints.add(new Translation2d(3, 1.25));
        }


        return TrajectoryGenerator.generateTrajectory(initialPose, interiorWaypoints, endingPose, config);

    }
}
