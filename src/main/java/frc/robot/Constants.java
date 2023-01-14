// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CanIds {
    public static final int kArmMotorId = 0;
    public static final int kWristMotorId = 1;

  }

  public static class ArmWristConstants {
    /* ---------------------------------------------------------------------------- 
      PID Constants */
    public static final double kArmKp = 1;
    public static final double kArmKi = 0;
    public static final double kArmKd = 0;
    public static final double kArmPositionTolerance = 1;
    public static final double kArmVelocityTolerance = 1;

    public static final double kWristKp = 0.5;
    public static final double kWristKi = 0;
    public static final double kWristKd = 0;
    public static final double kWristPositionTolerance = 1;
    public static final double kWristVelocityTolerance = 1;

    /* ---------------------------------------------------------------------------- 
      Constraint Constants */
    public static final double kArmMaxVelocity = 180; // degrees/s
    public static final double kArmMaxAcceleration = 360; // degrees/s^2
    public static final double kArmStartingPos = 60; // 60 degrees wrt hortizontal
    public static final double kArmMinAngle = -30; // degrees
    public static final double kArmMaxAngle = 150; // degrees

    public static final double kWristMaxVelocity = 180; // degrees/s
    public static final double kWristMaxAcceleration = 360; // degrees/s^2
    public static final double kWristStartingPos = 15; // 60 degrees wrt hortizontal
    public static final double kWristMinAngle = -180; // degrees
    public static final double kWristMaxAngle = 180; // degrees


    /* ---------------------------------------------------------------------------- 
      Physical Constants */    
    public static final double kArmMoi = 10; // kg * m^2
    public static final double kArmGearboxReduction = 80; // greater than 1 is a speed reduction
    public static final double kArmLength = Units.inchesToMeters(50); // meters
    public static final double kArmMass = 10; // kg

    public static final double kWristMoi = .25; // kg * m^2
    public static final double kWristGearboxReduction = 48; // greater than 1 is a speed reduction
    public static final double kWristLength = Units.inchesToMeters(15); // meters
    public static final double kWristMass = 2.5; // kg

    public static final double kArmEncoderDistPerPulse = 360.0 / 4096;
    public static final double kWristEncoderDistPerPulse = 360.0 / 4096;

  }
}
