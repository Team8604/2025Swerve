// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.EffectorConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunEffector extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  double effectorSpeed = 0, temp;
  boolean isCoral = false;

  public RunEffector() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.effector);
    temp = EffectorConstants.kIntakeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    RobotContainer.effector.setSpeed(effectorSpeed);

    if (temp < 0) {
      // would mean intake backing out
      isCoral = false;
    } else if (isCoral || RobotContainer.effector.getOutputCurrent() > 20) {
      // would mean already tripped or should based on current
      temp = 0;
      isCoral = true;
    }

    // sets speed to intake motor
    effectorSpeed = temp;
    RobotContainer.effector.setSpeed(effectorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.effector.setSpeed(0);
  }
}