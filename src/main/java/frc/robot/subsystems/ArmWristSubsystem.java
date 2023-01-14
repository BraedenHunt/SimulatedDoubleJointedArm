// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmWristConstants;
import frc.robot.Constants.CanIds;

public class ArmWristSubsystem extends SubsystemBase {

  private ProfiledPIDController m_armPidController;
  private ProfiledPIDController m_wristPidController;

  private CANSparkMax m_armMotor;
  private Encoder m_armEncoder;

  private CANSparkMax m_wristMotor;
  private Encoder m_wristEncoder;

  public boolean brakeEnabled = true;
  private boolean m_useBrakeAssist;


  /** Creates a new ArmSubsystem. */
  public ArmWristSubsystem(boolean useBrakeAssist) {

    this.m_useBrakeAssist = useBrakeAssist;

    // The arm ProfiledPIDController
    m_armPidController = new ProfiledPIDController(
            ArmWristConstants.kArmKp,
            ArmWristConstants.kArmKi,
            ArmWristConstants.kArmKd,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmWristConstants.kArmMaxVelocity, ArmWristConstants.kArmMaxAcceleration));

    m_wristPidController = new ProfiledPIDController(
                 ArmWristConstants.kWristKp,
            ArmWristConstants.kWristKi,
            ArmWristConstants.kWristKd,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmWristConstants.kWristMaxVelocity, ArmWristConstants.kArmMaxAcceleration));


    m_armPidController.setGoal(ArmWristConstants.kArmStartingPos);
    m_wristPidController.setGoal(ArmWristConstants.kWristStartingPos);

    m_armPidController.setTolerance(ArmWristConstants.kArmPositionTolerance, ArmWristConstants.kArmVelocityTolerance);
    m_wristPidController.setTolerance(ArmWristConstants.kWristPositionTolerance, ArmWristConstants.kWristVelocityTolerance);



    m_armMotor = new CANSparkMax(CanIds.kArmMotorId, MotorType.kBrushless);
    m_armEncoder = new Encoder(0, 1); // TODO: Determine channels
    m_armEncoder.setDistancePerPulse(ArmWristConstants.kArmEncoderDistPerPulse);

    m_wristMotor = new CANSparkMax(CanIds.kWristMotorId, MotorType.kBrushless);
    m_wristEncoder = new Encoder(2, 3); // TODO: Determine channels
    m_wristEncoder.setDistancePerPulse(ArmWristConstants.kWristEncoderDistPerPulse);



    enableBrake();
  }

  @Override
  public void periodic() {
    double armPidVoltage = m_armPidController.calculate(m_armEncoder.getDistance());

    if (m_armPidController.atGoal() && m_useBrakeAssist) {
      enableBrake();
    }
    else {
      disableBrake();
      double armVoltage = calculateArmFeedForwardVoltage() + armPidVoltage;
      m_armMotor.setVoltage(armVoltage);
    }

    double wristPidVoltage = m_wristPidController.calculate(m_wristEncoder.getDistance());

    double wristVoltage = calculateWristFeedForwardVoltage() + wristPidVoltage;
    m_wristMotor.setVoltage(wristVoltage);
  }

  public double calculateArmFeedForwardVoltage() {
    return 0;//1 * Math.cos(Units.degreesToRadians(m_armEncoder.getDistance())); // TODO: Calculate using MoI and setpoint angle of arm and MoI and setpoint angle of wrist
  }

  public double calculateWristFeedForwardVoltage() {
    return 0; // TODO: Calculate using setpoint angle of arm and MoI and setpoint angle of wrist
  }

  public void useBrakeAssist(boolean useBrake) {
    this.m_useBrakeAssist = useBrake;
    if (!useBrake) {
      disableBrake();
    }
  }

  public void enableBrake() {
    brakeEnabled = true;
    // TODO: Move servo to braking position

    m_armMotor.stopMotor();
  }

  public void disableBrake() {
    brakeEnabled = false;
    // TODO: Move servo to open position
  }

  public void setArmGoal(double degrees) {
    // TODO: Implement bounds
    m_armPidController.setGoal(degrees);
  }

  public void setWristGoal(double degrees) {
    // TODO: Implement bounds
    m_wristPidController.setGoal(degrees);
  }

  public void setArmWristGoal(double armDegrees, double wristDegrees) {
    setArmGoal(armDegrees);
    setWristGoal(wristDegrees);
  }

  public boolean armAtGoal() {
    return m_armPidController.atGoal();
  }

  public boolean wristAtGoal() {
    return m_wristPidController.atGoal();
  }

  public boolean armWristAtGoal() {
    return armAtGoal() && wristAtGoal();
  }

  public Command getGotoCommand(double armDegrees, double wristDegrees) {
    return Commands.runOnce(() -> setArmWristGoal(armDegrees, wristDegrees), this)
           .andThen(Commands.waitUntil(() -> armWristAtGoal()));
  }
  
  /* ---------------------- Simulation Code ---------------------- */
  private SingleJointedArmSim m_armSim;
  private SingleJointedArmSim m_wristSim;

  private EncoderSim m_armEncoderSim;
  private EncoderSim m_wristEncoderSim;

  private MechanismLigament2d m_armMechanism;
  private MechanismLigament2d m_wristMechanism;

  public void simulationInit() {
    var armGearbox = DCMotor.getNEO(6);
    var wristGearbox = DCMotor.getNeo550(1);

    m_armSim = new SingleJointedArmSim(
      armGearbox,
      ArmWristConstants.kArmGearboxReduction,
      SingleJointedArmSim.estimateMOI(ArmWristConstants.kArmLength, ArmWristConstants.kArmMass),
      ArmWristConstants.kArmLength,
      Units.degreesToRadians(ArmWristConstants.kArmMinAngle),
      Units.degreesToRadians(ArmWristConstants.kArmMaxAngle),
      ArmWristConstants.kArmMass,
      true,
      VecBuilder.fill(ArmWristConstants.kArmEncoderDistPerPulse/50) // Add noise with a std-dev of 1 tick
      );
    m_wristSim = new SingleJointedArmSim(
      wristGearbox,
      ArmWristConstants.kWristGearboxReduction,
      SingleJointedArmSim.estimateMOI(ArmWristConstants.kWristLength, ArmWristConstants.kWristMass),
      ArmWristConstants.kWristLength,
      Units.degreesToRadians(ArmWristConstants.kWristMinAngle),
      Units.degreesToRadians(ArmWristConstants.kWristMaxAngle),
      ArmWristConstants.kWristMass,
      false,
      VecBuilder.fill(ArmWristConstants.kArmEncoderDistPerPulse/50) // Add noise with a std-dev of 1 tick
        );

      m_armEncoderSim = new EncoderSim(m_armEncoder);
      m_wristEncoderSim = new EncoderSim(m_wristEncoder);

      Mechanism2d m_mech2d = new Mechanism2d(120, 120);
      MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 60, 15);
      MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 15, -90));
      m_armMechanism = m_armPivot.append(
        new MechanismLigament2d(
            "Arm",
            Units.metersToInches(ArmWristConstants.kArmLength),
            Units.radiansToDegrees(m_armSim.getAngleRads()),
            6,
            new Color8Bit(Color.kYellow)));

      m_wristMechanism = m_armMechanism.append(
                new MechanismLigament2d(
                    "Wrist",
                    Units.metersToInches(ArmWristConstants.kWristLength),
                    Units.radiansToDegrees(m_wristSim.getAngleRads()),
                    6,
                    new Color8Bit(Color.kRed)));

      SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(Math.max(Math.min(m_armMotor.getAppliedOutput(), RobotController.getBatteryVoltage()), -RobotController.getBatteryVoltage()));
    m_wristSim.setInput(Math.max(Math.min(m_wristMotor.getAppliedOutput(), RobotController.getBatteryVoltage()), -RobotController.getBatteryVoltage()));
    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);
    m_wristSim.update(0.020);


    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_armEncoderSim.setDistance(Units.radiansToDegrees(m_armSim.getAngleRads()));
    m_wristEncoderSim.setDistance(Units.radiansToDegrees(m_wristSim.getAngleRads()));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps() + m_wristSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_armMechanism.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    m_wristMechanism.setAngle(Units.radiansToDegrees(m_wristSim.getAngleRads()));
  }
}
