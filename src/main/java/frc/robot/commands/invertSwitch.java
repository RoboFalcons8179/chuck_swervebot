// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmControl;

public class invertSwitch extends CommandBase {

  private boolean f = true;

  /** Creates a new invertSwitch. */
  public invertSwitch(boolean isForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    f = isForward;
  }

  public invertSwitch() {
    // To not break existing code
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    ArmControl.elbowMotorLeft.set(Constants.kArm.kInvertElbow);
    ArmControl.shoulderMotorLeft.set(
      
      f ? //checks if the parameter on the parameterized constructor wants kInvertShoulderF or the other option, the ? is a fancy if/else statement

      Constants.kArm.kInvertShoulderF : 
      Constants.kArm.kInvertShoulderB

    );
  } // the goal of this is to eliminate the need for invertSwitchV2, im not going to delete it though because I'm not sure the intention of having seperate forward and backward commands -z

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
