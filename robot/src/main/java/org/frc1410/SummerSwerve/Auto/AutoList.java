package org.frc1410.SummerSwerve.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.ArrayList;

import static org.frc1410.SummerSwerve.util.Constants.MAX_ACCELERATION;
import static org.frc1410.SummerSwerve.util.Constants.MAX_SPEED;

public interface AutoList {

	ArrayList<PathPlannerTrajectory> OneGamePieceAuto =
		(ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
			"One game piece auto",
			new PathConstraints(MAX_SPEED, MAX_ACCELERATION));

}
