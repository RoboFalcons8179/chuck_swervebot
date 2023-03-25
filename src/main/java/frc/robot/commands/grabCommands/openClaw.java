// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class openClaw extends CommandBase {
  /** Creates a new openClaw. */


  // Directions - make sure to use the timeout command below after any time you
  // start this command.
  // .withTimeout(Constants.kGrabber.openTimeout)


  Grabber claw;
  public openClaw(Grabber claw_in) {
    // Use addRequirements() here to declare subsystem dependencies.
    claw = claw_in;

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    claw.openClawSubsystem();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // potentially add the wait here
    return false;
  }
}
