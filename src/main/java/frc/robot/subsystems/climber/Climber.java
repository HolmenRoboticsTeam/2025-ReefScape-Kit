// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setTargetAngle(double angle) {
    io.setTargetAngle(angle);
  }

  public double getTargetAngle() {
    return inputs.targetPosition;
  }

  public double getCurrentAngle() {
    return inputs.currentPosition;
  }

  public void resetEncoder() {
    io.resetEncoder();
  }
}
