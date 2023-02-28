// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmControl;

public class elbowMove extends CommandBase {

  public boolean f = true;

  /** Creates a new elbowMove. */
  public elbowMove() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public elbowMove(boolean isForward) {
    f = isForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ArmControl.elbowMotorLeft.set(

      f ?

      Constants.kArm.kElbowForward :
      Constants.kArm.kElbowBackward
    
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
