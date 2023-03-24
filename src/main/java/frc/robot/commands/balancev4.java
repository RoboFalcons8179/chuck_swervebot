// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class balancev4 extends SequentialCommandGroup {
  /** Creates a new balancev4. */
  public balancev4(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new WaitCommand(1).until(() -> s_Swerve.gyro.isMoving()),


    new balanceWrapper(s_Swerve, 1),


    new WaitCommand(1),

    new balanceWrapper(s_Swerve, .9),

    new WaitCommand(.8),

    new balanceWrapper(s_Swerve,0.9).withTimeout(0.8).andThen(new WaitCommand(0.75)),


    new balanceWrapper(s_Swerve,0.8).withTimeout(0.7).andThen(new WaitCommand(0.75)),

    new balanceWrapper(s_Swerve,0.75).withTimeout(0.4).andThen(new WaitCommand(0.7)),

    new balanceWrapper(s_Swerve,0.65).withTimeout(0.4).andThen(new WaitCommand(0.7)).repeatedly()


    );
  }
}
