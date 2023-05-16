package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;

//what is this for?
public class backUnTip extends CommandBase{
    private ArmControl arm;

    public backUnTip(ArmControl arm) {
      this.arm = arm;
    }

    //@Override
    //public void initialize() {

    //}

    @Override
    public void execute() {
        System.out.println("It did a thing");
        arm.setMaxOutput(1);
        arm.elbowOverride(-1);
    }
    

    @Override
    public void end(boolean interrupted) {
      arm.resetMaxOutput();
      arm.elbowOverride(0);
      arm.setHoldElbow(arm.elbowCurrentAngle());
    }


    @Override
    public boolean isFinished() {
      return false;
    }
}