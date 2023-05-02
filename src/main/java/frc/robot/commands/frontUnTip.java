package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


//what is this for?
public class frontUnTip extends CommandBase{


    public frontUnTip() {


    }

    // @Override
    // public void initialize() {

    // }

    @Override
    public void execute() {
        System.out.println("It did a thing");
    }
    

    @Override
    public void end(boolean interrupted) {


    }


    @Override
    public boolean isFinished() {
      return false;
    }
}