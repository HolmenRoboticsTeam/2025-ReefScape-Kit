// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private DCMotorSim climberMotor;

  private PIDController pidController;

  private double targetAngle;
  private double appliedVolts;

  public ClimberIOSim() {

    climberMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.motorGearbox, 0.025, ClimberConstants.motorToWheelRatio),
                ClimberConstants.motorGearbox);

    pidController = new PIDController(ClimberConstants.kSimP, ClimberConstants.kSimI, ClimberConstants.kSimD);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    climberMotor.setInputVoltage(appliedVolts);
    climberMotor.update(0.02);

    inputs.currentPosition = climberMotor.getAngularPositionRad();
    inputs.targetPosition = targetAngle;
    inputs.errorPosition = Math.abs(targetAngle - climberMotor.getAngularPositionRad());
    inputs.velocity = climberMotor.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = climberMotor.getCurrentDrawAmps();
  }

  public void setTargetAngle(double angle) {
    targetAngle = angle;
    double speed = pidController.calculate(climberMotor.getAngularPositionRad(), angle);
    double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);

    appliedVolts = volts;
  }
}
