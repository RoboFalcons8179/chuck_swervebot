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
  public void initialize() {

    System.out.println("GOING TO:");
    System.out.println(this.shoulderSet);
    System.out.println(this.elbowSet);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.goToElbowSetpoint(elbowSet);
    //arm.goToShoulderSetpoint(shoulderSet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    arm.lastElbowPosition = elbowSet;
    arm.lastShoulderPosition = shoulderSet;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(arm.shoulderMotorLeft.getSelectedSensorPosition() - arm.shoulderAngle2encoder(shoulderSet)) < 10000 && Math.abs(arm.elbowMotorLeft.getSelectedSensorPosition() - arm.elbowAngle2encoder(elbowSet)) < 200) {
      return true;
    }
    return false;
  }
}
