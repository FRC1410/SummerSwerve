package org.frc1410.chargedup2023.Commands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.frc1410.chargedup2023.util.Constants.*;

public class PathfindToPose extends Command {

	private final Drivetrain drivetrain;
	private final PathfindHolonomic pathfindHolonomic;

	public PathfindToPose(Drivetrain drivetrain) {

		List<Pose2d> shootingPoses = Arrays.asList(
			new Pose2d(1.36, 5.52, new Rotation2d(0)),
			new Pose2d(3, 3, new Rotation2d(0))
		);

		Pose2d curentRobotPose = drivetrain.getPoseMeters();
		Pose2d nearestPose = curentRobotPose.nearest(shootingPoses);

		pathfindHolonomic = new PathfindHolonomic(
			nearestPose,
			constraints,
			0.0,
			drivetrain::getPoseMeters,
			drivetrain::getRobotRelativeSpeeds,
			drivetrain::driveRobotRelative,
			holonomicPathFollowerConfig,
			0.0,
			drivetrain
		);

		this.drivetrain = drivetrain;
    }

	@Override
	public void initialize() {
		this.pathfindHolonomic.initialize();
	}

	@Override
	public void execute() {
		this.pathfindHolonomic.execute();
	}

	@Override
	public void end(boolean interrupted) {
		this.pathfindHolonomic.end(interrupted);
	}

	@Override
	public boolean isFinished() {

		return this.pathfindHolonomic.isFinished();
	}


}
