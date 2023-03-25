// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/** Add your docs here. */
public final class PPTRAJ {

    public final static PathConstraints defaultPathCon = new PathConstraints(2, 2);


    public final static PathPlannerTrajectory CenterRB = PathPlanner.loadPath("CENTERRB", defaultPathCon);

    public final static PathPlannerTrajectory CenterMO = PathPlanner.loadPath("CENTERMO", defaultPathCon);
    
    public final static PathPlannerTrajectory CenterPSTraj = PathPlanner.loadPath("CENTERPS", defaultPathCon);
    
    public final static PathPlannerTrajectory CenterS = PathPlanner.loadPath("CENTERS", defaultPathCon);
    
    public final static PathPlannerTrajectory CenterB = PathPlanner.loadPath("CENTERSB", defaultPathCon);
    
    public final static PathPlannerTrajectory CenterSPTraj = PathPlanner.loadPath("CENTERSP", defaultPathCon);

    public final static PathPlannerTrajectory LeaveC = PathPlanner.loadPath("LEAVEC", defaultPathCon);
    
    public final static PathPlannerTrajectory LeftBPPTraj = PathPlanner.loadPath("LEFTB", defaultPathCon);
    
    public final static PathPlannerTrajectory LeftP = PathPlanner.loadPath("LEFTP", defaultPathCon);
    
    public final static PathPlannerTrajectory LeftPS = PathPlanner.loadPath("LEFTPS", defaultPathCon);
    
    public final static PathPlannerTrajectory LeftSPPTraj = PathPlanner.loadPath("LEFTS", defaultPathCon);

    public final static PathPlannerTrajectory ReturnC = PathPlanner.loadPath("RETURNC", defaultPathCon);
    
    public final static PathPlannerTrajectory RightB = PathPlanner.loadPath("RIGHTB", defaultPathCon);
    
    public final static PathPlannerTrajectory RightP = PathPlanner.loadPath("RIGHTP", defaultPathCon);
    
    public final static PathPlannerTrajectory RightPS = PathPlanner.loadPath("RIGHTPS", defaultPathCon);
    
    public final static PathPlannerTrajectory RightS = PathPlanner.loadPath("RIGHTS", defaultPathCon);

    public final static PathPlannerTrajectory StraightL = PathPlanner.loadPath("STRAIGHTL", defaultPathCon);

    public final static PathPlannerTrajectory StraightLR = PathPlanner.loadPath("STRAIGHTLR", defaultPathCon);

    public final static PathPlannerTrajectory StraightC = PathPlanner.loadPath("STRAIGHTC", defaultPathCon);

    public final static PathPlannerTrajectory StraightR = PathPlanner.loadPath("STRAIGHTR", defaultPathCon);

    public final static PathPlannerTrajectory StraightBR = PathPlanner.loadPath("STRAIGHTBR", defaultPathCon);

    public final static PathPlannerTrajectory compLeftRed = PathPlanner.loadPath("COMPLEFTRED", defaultPathCon);

    public final static PathPlannerTrajectory compRightRed = PathPlanner.loadPath("COMPRIGHTRED", defaultPathCon);

    public final static PathPlannerTrajectory compMiddleRed = PathPlanner.loadPath("COMPSTRAIGHTRED", defaultPathCon);

    public final static PathPlannerTrajectory compLeftBlue = PathPlanner.loadPath("COMPLEFTBLUE", defaultPathCon);

    public final static PathPlannerTrajectory compRightBlue = PathPlanner.loadPath("COMPRIGHTBLUE", defaultPathCon);
    
    public final static PathPlannerTrajectory compMiddleBlue = PathPlanner.loadPath("COMPSTRAIGHTBLUE", defaultPathCon);

    public final static PathPlannerTrajectory newPath = PathPlanner.loadPath("New Path", defaultPathCon);

    public final static PathPlannerTrajectory balance15M1 = PathPlanner.loadPath("15BALANCEM1", defaultPathCon);

    public final static PathPlannerTrajectory balance15M2 = PathPlanner.loadPath("15BALANCEM2", defaultPathCon);
}
