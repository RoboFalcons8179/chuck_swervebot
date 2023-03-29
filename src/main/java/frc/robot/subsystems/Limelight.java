// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  NetworkTable table;


  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // This code needs to be testing. will pass it back to the driver pipeline if the pipeline is busted.
    // int pipe = (int) table.getEntry("getpipe").getInteger(0);

    // if (!(pipe == 1 || pipe == 2)) {
    //   this.goToDriverCam();
    // }

  }

  public void reflect_init() {
    // reflect pipeline is pipe 0.
    table.getEntry("pipeline").setNumber(0);
    table.getEntry("tx").getDouble(0);


  }

  public double get_reflectTX() {
    return table.getEntry("tx").getDouble(0);
  }


  public void april_init() {
    // apriltag pipeline is pipe 1.
    table.getEntry("pipeline").setNumber(1);
    table.getEntry("tx").getDouble(0);


  }

  public double get_TagTX() {
    return table.getEntry("tx").getDouble(0);
  }

  public double get_TagTY() {
    return table.getEntry("ty").getDouble(0);

  }

  public void goToDriverCam() {
    // driver pipe is pipe 2.
    table.getEntry("pipeline").setNumber(2);


  }


}
