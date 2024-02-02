package org.frc1410.chargedup2023.Commands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import static org.frc1410.chargedup2023.util.Constants.*;

public class PathFindToNearestPose extends Command {

    private PathfindHolonomic pathfindHolonomic;

	private final Drivetrain drivetrain;

	public PathFindToNearestPose(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
    }

	@Override
	public void initialize() {

		Pose2d currentRobotPose = drivetrain.getPoseMeters();
		Pose2d nearestPose = currentRobotPose.nearest(shootingPoses.stream().map(shootingPositions -> shootingPositions.pose2d).toList());

		pathfindHolonomic = new PathfindHolonomic(
			nearestPose,
			constraints,
			0.0,
			drivetrain::getPoseMeters,
			drivetrain::getRobotRelativeSpeeds,
			drivetrain::driveRobotRelative,
			pathfindPathFollowerConfig,
			0.0,
			drivetrain
		);

		this.pathfindHolonomic.initialize();
	}

	@Override
	public void execute() {
		if(this.pathfindHolonomic != null) {
			this.pathfindHolonomic.execute();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if(this.pathfindHolonomic != null) {
			this.pathfindHolonomic.end(interrupted);
		}
	}

	@Override
	public boolean isFinished() {
		if(this.pathfindHolonomic != null) {
			return this.pathfindHolonomic.isFinished();
		} else {
			return false;
		}
	}
}
