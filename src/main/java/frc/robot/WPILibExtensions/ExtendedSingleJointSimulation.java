// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.WPILibExtensions;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ExtendedSingleJointSimulation extends SingleJointedArmSim {
   // The gearbox for the arm.
  private final DCMotor m_gearbox;

  // The gearing between the motors and the output.
  private final double m_gearing;

  // The length of the arm.
  private final double m_r;

  // The minimum angle that the arm is capable of.
  private final double m_minAngle;

  // The maximum angle that the arm is capable of.
  private final double m_maxAngle;

  // The mass of the arm.
  private final double m_armMass;

  // Whether the simulator should simulate gravity.
  private final BooleanSupplier m_simulateGravity;

  // The offset of the gravity vector in radians (useful for multijointed arms)
  private final DoubleSupplier m_gravityVectorOffset;
   
    /**
   * Creates a simulated arm mechanism.
   *
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
   * @param armLengthMeters The length of the arm.
   * @param minAngleRads The minimum angle that the arm is capable of.
   * @param maxAngleRads The maximum angle that the arm is capable of.
   * @param armMassKg The mass of the arm.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public ExtendedSingleJointSimulation(
    DCMotor gearbox,
    double gearing,
    double jKgMetersSquared,
    double armLengthMeters,
    double minAngleRads,
    double maxAngleRads,
    double armMassKg,
    BooleanSupplier simulateGravity,
    Matrix<N1, N1> measurementStdDevs) {
  super(gearbox, 
      gearing, 
      jKgMetersSquared, 
      armLengthMeters, 
      minAngleRads,
      maxAngleRads,
      armMassKg,
      simulateGravity.getAsBoolean(),
      measurementStdDevs);

  m_gearbox = gearbox;
  m_gearing = gearing;
  m_r = armLengthMeters;
  m_minAngle = minAngleRads;
  m_maxAngle = maxAngleRads;
  m_armMass = armMassKg;
  m_simulateGravity = simulateGravity;
  m_gravityVectorOffset = () -> 0;
}

    /**
   * Creates a simulated arm mechanism.
   *
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
   * @param armLengthMeters The length of the arm.
   * @param minAngleRads The minimum angle that the arm is capable of.
   * @param maxAngleRads The maximum angle that the arm is capable of.
   * @param armMassKg The mass of the arm.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param gravityVectorOffset The offset of the gravity vector in radians (useful for multijointed arms)
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public ExtendedSingleJointSimulation(
      DCMotor gearbox,
      double gearing,
      double jKgMetersSquared,
      double armLengthMeters,
      double minAngleRads,
      double maxAngleRads,
      double armMassKg,
      BooleanSupplier simulateGravity,
      DoubleSupplier gravityVectorOffset,
      Matrix<N1, N1> measurementStdDevs) {
    super(gearbox, 
        gearing, 
        jKgMetersSquared, 
        armLengthMeters, 
        minAngleRads,
        maxAngleRads,
        armMassKg,
        simulateGravity.getAsBoolean(),
        measurementStdDevs);

    m_gearbox = gearbox;
    m_gearing = gearing;
    m_r = armLengthMeters;
    m_minAngle = minAngleRads;
    m_maxAngle = maxAngleRads;
    m_armMass = armMassKg;
    m_simulateGravity = simulateGravity;
    m_gravityVectorOffset = gravityVectorOffset;
  }


  /**
   * Updates the state of the arm.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Horizontal case:
    // Torque = F * r = I * alpha
    // alpha = F * r / I
    // Since F = mg,
    // alpha = m * g * r / I
    // Finally, multiply RHS by cos(theta) to account for the arm angle
    // This acceleration is added to the linear system dynamics x-dot = Ax + Bu
    // We therefore find that f(x, u) = Ax + Bu + [[0] [m * g * r / I *
    // cos(theta)]]
    Matrix<N2, N1> updatedXhat =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity.getAsBoolean()) {
                xdot =
                    xdot.plus(
                        VecBuilder.fill(
                            0,
                            m_armMass
                                * m_r
                                * -9.8
                                * 3.0
                                / (m_armMass * m_r * m_r)
                                * Math.cos(x.get(0, 0) + m_gravityVectorOffset.getAsDouble())));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collision after updating xhat
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minAngle, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxAngle, 0);
    }
    return updatedXhat;
  }
}
