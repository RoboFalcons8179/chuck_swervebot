// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;

public class goto30 extends CommandBase {
  /** Creates a new goto30. */

  ArmControl arm;

  public goto30(ArmControl arm_in) {
    // Use addRequirements() here to declare subsystem dependencies.

    arm = arm_in;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.goToShoulderSetpoint(10);
    arm.goToElbowSetpoint(90+30);

    System.out.println("--------------");
    System.out.println(10);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(arm.shoulderEncoder2Angle(arm.shoulderMotorLeft.getSelectedSensorPosition() - 10)) < 2)
        && (Math.abs(arm.elbowEncoder2Angle(arm.elbowMotorLeft.getSelectedSensorPosition()) - 120) < 2)) {
      return true;
    }
    return false;
  }
}
