package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class doAThing implements Command {

    public doAThing() {
        System.out.println("*******,,,,,,,,,,,,,\n,***,,,,,........,,,\n,,,,,,,,(#.....//.,,\n,,,,,,,,/,*.(*/((,,,\n*,,,,,,*#*##*,#,,,,,\n,,,,**/&%(*(,,,/((,,\n*,,,,,*/##/,,,,,,,,,\n***********,,,,,,,,,");
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
