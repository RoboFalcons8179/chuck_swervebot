// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;

public class defaultArm extends CommandBase {
  /** Creates a new defaultArm. */

  public ArmControl arm;
  
  public defaultArm(ArmControl arm_in) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.arm = arm_in;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.lastElbowPosition = arm.elbowMotorLeft.getSelectedSensorPosition();
    arm.lastShoulderPosition = arm.shoulderMotorLeft.getSelectedSensorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /// TO ADD: IF SHOULDER HIGH, GO DOWN SLOWLY. TIM TO ADD
    this.arm.holdPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.arm.shoulderMotorRight.follow(arm.shoulderMotorLeft);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
