package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class teste implements Command{


// attempt to read alliance color and number
public enum Alliance {
    Red1,
    Blue1,
    Red2,
    Blue2,
    Red3,
    Blue3,
    Invalid,
}

public static void getAlliance() {
             
    AllianceStationID allianceID = DriverStationJNI.getAllianceStation();
    String test = "test";
    //numbers 1-3 are for red and numbers 4-6 are for blue
    //waiting for Tim
    boolean isRedAlliance = false;
    int stationNumber =1;
    switch (allianceID) {                 
        case Red1:                     
            isRedAlliance = true;
            stationNumber = 1;
            System.out.println(test);
            break;
       case Red2:                    
           isRedAlliance = true;
           stationNumber = 2;
           break;
        case Red3:                     
            isRedAlliance = true;
            stationNumber = 3;
            break;
        case Blue1:                     
            isRedAlliance = false;
            stationNumber = 4;
            break;
        case Blue2:                     
            isRedAlliance = false;
            stationNumber = 5;
            break;
        case Blue3:                     
            isRedAlliance = false;
            stationNumber = 6;
            break;
          default:
            //return Alliance.Invalid;
        }
    }

@Override
public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    return null;
}
}