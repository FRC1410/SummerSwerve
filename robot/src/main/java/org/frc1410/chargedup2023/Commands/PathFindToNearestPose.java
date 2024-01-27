package org.frc1410.chargedup2023.Commands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;
import org.frc1410.chargedup2023.util.ShootingPositions;

import java.util.Arrays;
import java.util.List;

import static org.frc1410.chargedup2023.util.Constants.*;

public class PathFindToNearestPose extends Command {

    private final PathfindHolonomic pathfindHolonomic;

	public PathFindToNearestPose(Drivetrain drivetrain) {

		List<ShootingPositions> shootingPositions = Arrays.asList(
			new ShootingPositions(new Pose2d(1.36, 5.52, Rotation2d.fromDegrees(0)), 0),
			new ShootingPositions(new Pose2d(1.94, 6.15, Rotation2d.fromDegrees(20.5)), 0),
			new ShootingPositions(new Pose2d(1.88, 5, Rotation2d.fromDegrees(-19.6)), 0)
		);

		Pose2d currentRobotPose = drivetrain.getPoseMeters();
		Pose2d nearestPose = currentRobotPose.nearest(shootingPositions.stream().map(position -> position.pose2d).toList());

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
