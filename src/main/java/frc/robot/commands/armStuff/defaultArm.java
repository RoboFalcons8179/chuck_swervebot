// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;
import edu.wpi.first.wpilibj.Joystick;

public class defaultArm extends CommandBase {
  /** Creates a new defaultArm. */

  public ArmControl arm;
  public Joystick board;
  
  public defaultArm(ArmControl arm_in, Joystick board_in) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.arm = arm_in;
    this.board = board_in;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.lastElbowPosition = ArmControl.elbowMotorLeft.getSelectedSensorPosition();
    arm.lastShoulderPosition = ArmControl.shoulderMotorLeft.getSelectedSensorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /// TO ADD: IF SHOULDER HIGH, GO DOWN SLOWLY. TIM TO ADD

    boolean forward = false;
    boolean reverse = false;
    boolean gripUp = false;
    boolean gripDown = false;

  /*if ((int) board.getY() == 1){ forward = true;
    System.out.print("Forward ");
    System.out.print(forward);
    arm.goToElbowSetpoint(arm.elbowCurrentAngle() + 5);
  };


  if ((int) board.getY() == -1){ reverse = true;
    System.out.print("Reverse ");
    System.out.print(reverse);
    arm.goToElbowSetpoint(arm.elbowCurrentAngle() - 5);
  };

  if ((int) board.getX() == 1){ gripUp = true;
    System.out.print("GripUp ");
    System.out.print(gripUp);
    arm.goToShoulderSetpoint(arm.shoulderCurrentAngle() + 5);
  };

  if ((int) board.getX() == -1){ gripDown = true;
    System.out.print("GripDown ");
    System.out.print(gripDown);
    arm.goToShoulderSetpoint(arm.shoulderCurrentAngle() - 5);
  };*/

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
