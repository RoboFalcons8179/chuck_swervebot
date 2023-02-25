// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmControl;

public class cubeMedium extends CommandBase {

  public ArmControl arm;
  /** Creates a new coneLow. */
  public cubeMedium(ArmControl arm_in) {
    

    this.arm = arm_in;

    addRequirements(arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmControl.elbowMotorLeft.set(Constants.kArm.kElbowIn);
    ArmControl.shoulderMotorLeft.set(Constants.kArm.kShoulderMediumCube);
    ArmControl.elbowMotorLeft.set(Constants.kArm.kElbowMediumCube);
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
