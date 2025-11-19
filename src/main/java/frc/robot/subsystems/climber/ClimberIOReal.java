// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.MotorConfigs.WristConfig;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {

  private SparkMax climberMotor;
  private RelativeEncoder encoder;

  private PIDController pidController;

  private double targetAngle = 0.0;
  private double appliedVolts = 0.0;

  public ClimberIOReal() {
    climberMotor = new SparkMax(ClimberConstants.motorID, MotorType.kBrushless);
    encoder = climberMotor.getEncoder();

    tryUntilOk(
        climberMotor,
        5,
        () ->
            climberMotor.configure(
                WristConfig.wristConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(climberMotor, 5, () -> encoder.setPosition(0.0));

    pidController =
        new PIDController(ClimberConstants.realP, ClimberConstants.realI, ClimberConstants.realD);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.currentPosition = encoder.getPosition();
    inputs.targetPosition = targetAngle;
    inputs.errorPosition = Math.abs(targetAngle - encoder.getPosition());
    inputs.velocity = encoder.getVelocity();
    inputs.appliedVolts = climberMotor.getAppliedOutput() * climberMotor.getBusVoltage();
    inputs.currentAmps = climberMotor.getOutputCurrent();
  }

  public void setTargetAngle(double angle) {
    targetAngle = angle;
    double speed = pidController.calculate(encoder.getPosition(), angle);
    double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);

    climberMotor.setVoltage(volts);
  }
}
