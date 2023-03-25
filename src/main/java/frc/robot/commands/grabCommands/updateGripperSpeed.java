// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Grabber;

public class updateGripperSpeed extends CommandBase {
  /** Creates a new updateGripperSpeed. */
  Grabber claw;

  DoubleSupplier leftTriggerSupplier;

  DoubleSupplier rightTriggerSupplier;

  public updateGripperSpeed(Grabber claw, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightTriggerValue = rightTriggerSupplier.getAsDouble();
    double leftTriggerValue = leftTriggerSupplier.getAsDouble();

    if (Math.abs(rightTriggerValue) < 0.1) {
      rightTriggerValue = 0;
    }

    if (Math.abs(leftTriggerValue) < 0.1) {
      leftTriggerValue = 0;
    }

    claw.setSpeed((rightTriggerValue - leftTriggerValue) * 5);
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
