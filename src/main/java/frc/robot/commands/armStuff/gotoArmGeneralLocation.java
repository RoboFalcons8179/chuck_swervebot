// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;


public class gotoArmGeneralLocation extends CommandBase {
  /** Creates a new gotoArmGeneralLocation. */

  ArmControl arm;
  double shoulderSet;
  double elbowSet;

  public gotoArmGeneralLocation(ArmControl arm_in, double shoulderSetpoint, double ElbowSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm_in;
    this.shoulderSet = shoulderSetpoint;
    this.elbowSet = ElbowSetpoint;

    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.goToElbowSetpoint(arm.elbowAngle2encoder(elbowSet));
    arm.goToShoulderSetpoint(arm.shoulderAngle2encoder(shoulderSet));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(arm.shoulderMotorLeft.getSelectedSensorPosition() - shoulderSet) < 10000 && Math.abs(arm.elbowMotorLeft.getSelectedSensorPosition() - elbowSet) < 200) {
      return true;
    }
    return false;
  }
}
