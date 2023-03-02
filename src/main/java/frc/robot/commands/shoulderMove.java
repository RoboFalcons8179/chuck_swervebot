// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ArmControl; 

public class shoulderMove extends CommandBase {

  public boolean f = true;

  /** Creates a new shoulderMove. */
  public shoulderMove() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public shoulderMove (boolean isForward) {
    f = isForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Command For Moving Shoulder 
    ArmControl.shoulderMotorLeft.set(

    f ?

    Constants.kArm.kShoulderForward:
    Constants.kArm.kShoulderBackward

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

public Command andThen(Object println) {
    return null;
}
}
