// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double currentPosition = 0.0;
    public double targetPosition = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double errorPosition = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setTargetAngle(double angle) {}

  public default void resetEncoder() {}
  ;
}
