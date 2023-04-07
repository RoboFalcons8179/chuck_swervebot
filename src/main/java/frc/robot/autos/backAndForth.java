// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.TeleopSwerve;
import java.util.function.BooleanSupplier;
import frc.robot.commands.armStuff.*;
import frc.robot.commands.grabCommands.closeClaw;
import frc.robot.commands.grabCommands.openClaw;
import frc.robot.commands.grabCommands.squeezeClaw;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class backAndForth extends SequentialCommandGroup {
  /** Creates a new backAndForth. */
  public backAndForth(Swerve s_Swerve, ArmControl arm, Grabber claw) {

    addCommands(
    
 
new closeClaw(claw).until(() -> claw.grabberMotor.getStatorCurrent() > 4),
    

new updateHoldPosition(() -> -6, () -> 45, arm).repeatedly().until(() -> arm.elbowIsAtSetpoint()),// elbow is out if the way


new updateHoldPosition(() -> 115, () -> 45, arm).repeatedly().until(() -> arm.shoulderIsAtDegree(60)), // shoulder poition is high enough for us to move the elbow


new updateHoldPosition(() -> 115, () -> 150, arm),

new WaitCommand(4).until(() -> arm.isAtSetpoints()),

new openClaw(claw).withTimeout(1), // do quick open to get it out


new updateHoldPosition(() -> 115, () -> 65, arm).repeatedly().until(() -> arm.elbowIsAtSetpoint())
.alongWith(new squeezeClaw(claw).withTimeout(.6)), // doWith or Parrallel
                            // ^potentally tto 70

new updateHoldPosition(() -> -6, () -> 45, arm),

//new WaitCommand(4).until(() -> arm.isAtSetpoints()),

new InstantCommand(() -> claw.stop()),

new InstantCommand(()-> s_Swerve.zeroGyro()),
 
new InstantCommand(() -> s_Swerve.resetModulesToAbsolute())
);
      



//     //addCommands( new doTrajectory(s_Swerve, traj.makeZFlipped));

  }
}


