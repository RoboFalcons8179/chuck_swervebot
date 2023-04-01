// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armStuff.updateHoldPosition;
import frc.robot.commands.grabCommands.closeClaw;
import frc.robot.commands.grabCommands.squeezeClaw;
import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class backAndForthCleanup extends SequentialCommandGroup {
  /** Creates a new backAndForthCleanup. */
  public backAndForthCleanup(ArmControl arm, Grabber claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new squeezeClaw(claw).withTimeout(0.1),
      // new closeClaw(claw).withTimeout(0.5)
    
      //(new updateHoldPosition(() -> 60, () -> 65, arm).repeatedly()),//.until(() -> arm.elbowCurrentAngle() > (90)), // doWith or Parrallel

      new updateHoldPosition(() -> -6, () -> 45, arm),

      //new WaitCommand(4).until(() -> arm.isAtSetpoints()),

      new InstantCommand(() -> claw.stop())


    );
  }
}
