// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ClimberCommands {

  public static Command climberToTarget(
      Climber climber, DoubleSupplier targetAngle, boolean allowEndCondition) {
    Command returnCommand =
        Commands.runEnd(
                () -> {
                  climber.setTargetAngle(targetAngle.getAsDouble());
                },
                () -> {
                  climber.setTargetAngle(climber.getCurrentAngle());
                },
                climber)
            .until(
                allowEndCondition
                    ? () ->
                        Math.abs(climber.getCurrentAngle() - targetAngle.getAsDouble())
                            < ClimberConstants.angleErrorAllowed
                    : () -> false)
            .withName("climberToTarget");

    return returnCommand;
  }
}
