// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BalanceBasic;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.subsystems.Swerve;

public class autoBalance {
    private int state;
    private int debounceCount;
    private double robotSpeedSlow;
    private double robotSpeedMedium;
    private double robotSpeedFast;
    private double robotSpeedCreep;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private double singleTapTime;
    private double scoringBackUpTime;
    private double doubleTapTime;
    private double angleUp;
    private double scaleCreepSpeed;
    private double DscaleCreepSpeed;
    private double lastCreepSpeed;
    private double dCreepSpeed;

    private boolean done;

    private AHRS gyro;

    private Swerve s_Swerve;

    public boolean goToZeroAngle;

    public autoBalance(Swerve s_Swerve) {

        this.s_Swerve = s_Swerve;
        this.gyro = s_Swerve.gyro;

        goToZeroAngle = false;

        state = 0;
        debounceCount = 0;

        /**********
         * CONFIG *
         **********/
        // Speed the robot drived while scoring/approaching station, default = 0.4
        robotSpeedFast = 0.75;

        robotSpeedMedium = 0.6;//535;

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow =0.185; //0.195;

        // Final correction speed
        robotSpeedCreep = 0.15;

        scaleCreepSpeed = 0.0125;
        DscaleCreepSpeed = 0.03;
        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = 13;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = 10.7; // from 9.1

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = 0.15;

        // Amount of time to drive towards to scoring target when trying to bump the
        // game piece off
        // Time it takes to go from starting position to hit the scoring target
        singleTapTime = 0.4;

        // Amount of time to drive away from knocked over gamepiece before the second
        // tap
        scoringBackUpTime = 0.2;

        // Amount of time to drive forward to secure the scoring of the gamepiece
        doubleTapTime = 0.3;

        // used to exit the command that calls this
        done = false;

        // default start angle
        angleUp = Math.PI;

    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double autoBalanceRoutine() {

        dCreepSpeed = lastCreepSpeed - scaleCreepSpeed* getTilt();
        lastCreepSpeed = scaleCreepSpeed* getTilt();


        // System.out.print(state);
        // System.out.print("    ");
        // System.out.print(getTilt());
        // System.out.println("");

        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (Math.abs(getTilt()) > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    
                    // angleUp = s_Swerve.pointingUpAngle();


                    System.out.println("Ending High");

                    return robotSpeedMedium;


                }
                return robotSpeedFast;

            // // driving up charge station kind of fast for a set time
            case 1:
                if (true) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(1.3)) { //1.16 <----mEDIUM SPEED TIMING
                    state = 2;
                    debounceCount = 0;

                    System.out.println("Ending Medium");


                    return 0;
                }
                return robotSpeedMedium;  
            
            // approach, driving up charge station, drive slower, stopping when level
            case 2:
            if (Math.abs(getTilt()) < levelDegree) {
                debounceCount++;
                return 0;
            }
            if (debounceCount > secondsToTicks(0.02)) {
                state = 3;
                debounceCount = 0;

                System.out.println("Ending Low");

                goToZeroAngle = true;

                return 0;
            }
            return robotSpeedSlow; 
            // on charge station, stop motors and wait for end of auto
            case 3:
                if (Math.abs(getTilt()) <= Math.abs(levelDegree)) {
                    debounceCount++;
                    state = 4;
                    return 0;

                }
                if (debounceCount > 3 * secondsToTicks(debounceTime)) {
                    state = 4;
                    debounceCount = 0;
                    done = true;

                    System.out.println("Done");
                    return 0;
                }

                double sign = -1;

                if (getTilt() <= -levelDegree) {
                     sign = 1;
                }
                

                // return sign*scaleCreepSpeed * getTilt();


                if (getTilt() >= levelDegree) {
                    return -robotSpeedCreep;
                } else if (getTilt() <= -levelDegree) {
                    return robotSpeedCreep;
                }





            case 4:
                return 0;
        }
        return 0;
    }

    public boolean getDone() {
        return done;
    }

    public double getDriveUpAngle() {

        if (state >= 1 ) {
            return angleUp;
        }
        else {
            return Math.PI;
        }
        
    }
}