// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armStuff;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmControl;

public class updateHoldPosition extends InstantCommand {
  private DoubleSupplier holdShoulder;
  private DoubleSupplier holdElbow;
  private ArmControl arm;
  /** Creates a new updateHoldPosition. */
  public updateHoldPosition(DoubleSupplier setShoulder, DoubleSupplier setElbow, ArmControl arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    holdElbow = setElbow;
    holdShoulder = setShoulder;
    
    this.arm = arm;
    System.out.println(setShoulder + " " + holdShoulder);
    System.out.println(setElbow + " " + holdElbow);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setHoldElbow(holdElbow.getAsDouble());
    arm.setHoldShoulder(holdShoulder.getAsDouble());
  }

  // Called when the command is initially scheduled
}
