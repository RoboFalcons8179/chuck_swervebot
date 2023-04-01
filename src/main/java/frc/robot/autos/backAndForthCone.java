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
public class backAndForthCone extends SequentialCommandGroup {
  /** Creates a new backAndForth. */
  public backAndForthCone(Swerve s_Swerve, ArmControl arm, Grabber claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

new updateHoldPosition(() -> -6, () -> 65, arm),

new squeezeClaw(claw).withTimeout(0.55),//until(() -> claw.grabberMotor.getStatorCurrent() > 4),
    
//new doTrajectory(s_Swerve, traj.makeZ),

// elbow is out if the way

// new WaitCommand(4).until(() -> arm.isAtSetpoints()),

new updateHoldPosition(() -> 125, () -> 45, arm).repeatedly().until(() -> arm.shoulderCurrentAngle() >  (60)), // shoulder poition is high enough for us to move the elbow

//new WaitCommand(4).until(() -> arm.isAtSetpoints()),

new updateHoldPosition(() -> 125, () -> 155, arm),

new WaitCommand(4).until(() -> arm.isAtSetpoints()),

new openClaw(claw).withTimeout(1), // do quick open to get it out

new squeezeClaw(claw).withTimeout(0.5),

// From Tim - TEST THIS LINE. IT NEEDS TO TRIGGER US TO DRIVE. you might have to change this degree\/
new updateHoldPosition(() -> 125, () -> 65, arm).repeatedly().until(() -> arm.elbowCurrentAngle() > (100)),

new closeClaw(claw).withTimeout(0.002)

);


// left =.75
// right = -.75
// new TeleopSwerve(
// s_Swerve, 
// () -> .8, 
// () -> 0, 
// () -> 0, 
// () -> true,
// () -> false        
// ).withTimeout(3.5),//3.5
// new TeleopSwerve(
// s_Swerve, 
// () -> 0, 
// () -> 0, 
// () -> 0, 
// () -> true,
// () -> false        
// ).withTimeout(1),//2
// new TeleopSwerve(
// s_Swerve, 
// () -> -.8, 
// () -> 0, 
// () -> 0, 
// () -> true,
// () -> false          
// ).withTimeout(2.5),//2.3
// new TeleopSwerve(
// s_Swerve, 
// () -> 0, 
// () -> 0, 
// () -> 0, 
// () -> true,
// () -> false        
// ).withTimeout(5)

      



//     //addCommands( new doTrajectory(s_Swerve, traj.makeZFlipped));

  }
}


