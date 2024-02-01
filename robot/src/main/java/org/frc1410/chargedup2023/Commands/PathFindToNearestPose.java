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

    private PathfindHolonomic pathfindHolonomic;

	private final Drivetrain drivetrain;
	private Pose2d nearestPose;

	private List<Pose2d> shootingPoses = Arrays.asList(
		new Pose2d(1.36, 5.52, Rotation2d.fromDegrees(0)),
		new Pose2d(2.26, 4.81, Rotation2d.fromDegrees(20.5)),
		new Pose2d(1.88, 5, Rotation2d.fromDegrees(-19.6)),
		new Pose2d(3.31, 5.42, Rotation2d.fromDegrees(0)),
		new Pose2d(2.93, 6.21, Rotation2d.fromDegrees(12.77))
	);

	public PathFindToNearestPose(Drivetrain drivetrain) {

		this.drivetrain = drivetrain;

//		pathfindHolonomic = new PathfindHolonomic(
//			nearestPose,
//			constraints,
//			0.0,
//			drivetrain::getPoseMeters,
//			drivetrain::getRobotRelativeSpeeds,
//			drivetrain::driveRobotRelative,
//			pathfindPathFollowerConfig,
//			0.0,
//			drivetrain
//		);

    }

	@Override
	public void initialize() {
		nearestPose = drivetrain.getPoseMeters().nearest(shootingPoses);

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
