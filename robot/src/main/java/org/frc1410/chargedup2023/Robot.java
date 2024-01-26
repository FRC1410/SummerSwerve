package org.frc1410.chargedup2023;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

//import org.frc1410.chargedup2023.Subsystems.Camera;
import org.frc1410.chargedup2023.Subsystems.Camera;
import org.frc1410.chargedup2023.util.Constants;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.AutoSelector;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.Task;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import com.pathplanner.lib.commands.PathPlannerAuto;

import static org.frc1410.chargedup2023.util.Constants.*;

import org.frc1410.chargedup2023.Commands.DriveLooped;
import org.frc1410.chargedup2023.Commands.LockDrivetrainHeld;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

public final class Robot extends PhaseDrivenRobot {

	//<editor-fold desc="Controllers">
	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.2);
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER, 0.2);

	//</editor-fold>

	//<editor-fold desc="Auto Selector">
	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));
	private final Camera camera = subsystems.track(new Camera());

	{
		var layout = """
		[{
			"tabName": "Drive",
			"id": "drive",

			"components": [{
				"type": "string_select",
				"title": "Auto Selection",
				"layout": {
					"pos": [1, 1],
					"size": [2, 1]
				},
				"topics": ["Auto/Choices", "Auto/Selection"]
			}, {
				"type": "clock",
				"title": "Game Time",
				"layout": {
					"pos": [3, 1],
					"size": [2, 1]
				},
				"topics": ["FMSInfo/GameTime"]
			}, {
				"type": "node_select",
				"title": "Selected Node",
				"layout": {
					"pos": [5, 1],
					"size": [1, 1]
				},
				"topics": ["Drivetrain/Scoring Pose Index"]
			}, {
				"type": "boolean",
				"title": "LBork Line Break",
				"layout": {
					"pos": [6, 1],
					"size": [1, 1]
				},
				"topics": ["LBork/Line Break"]
			}]
		}]""";

		// grid, line break, auto, time
		var pub = NetworkTables.PublisherFactory(nt.getTable("viridian"), "layout", layout);
	}

	private final AutoSelector autoSelector = new AutoSelector()
		.add("PR-B#-A# (3pS)", () -> new PathPlannerAuto("PR-B#-A# (3pS)"))
		.add("PR-B#-C# (3pS)", () -> new PathPlannerAuto("PR-B#-C# (3pS)"))
		.add("PR-A# (1pA)", () -> new PathPlannerAuto("PR-A# (1pA)"));
//		.add("ENGAGE", () -> new Engage(this.drivetrain));
//		.add("FORWARD", () -> new Forward(this.drivetrain));
		// .add("NEW", () -> new PathPlannerAuto("New Path"));

	{
		var profiles = new String[autoSelector.getProfiles().size()];
		for (var i = 0; i < profiles.length; i++) {
			profiles[i] = autoSelector.getProfiles().get(i).name();
		}

		var pub = NetworkTables.PublisherFactory(table, "Choices", profiles);
		pub.accept(profiles);
	}


	private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(table, "Profile",
			autoSelector.getProfiles().isEmpty() ? "" : autoSelector.getProfiles().get(0).name());

	private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(table, autoPublisher.getTopic());


    //</editor-fold>

	@Override
	public void autonomousSequence() {
		// this.drivetrain.odometry.resetPosition(null, null, null);

		// this.drivetrain.zeroYaw();
		// this.drivetrain.resetPose(new Pose2d());


		
		// PathPlannerTrajectory examplePath = PathPlanner.loadPath("3p", new PathConstraints(1, 1));
		// if (examplePath == null) {
		// 	System.out.println("PATH NULL");
		// }
		// Command fullAuto = this.drivetrain.autoBuilder.fullAuto(examplePath);


		 NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
		 String autoProfile = autoSubscriber.get();
		 var autoCommand = autoSelector.select(autoProfile);

//		scheduler.scheduleAutoCommand(new PathPlannerAuto("PR-B#-A# (3p)"));

		scheduler.scheduleAutoCommand(autoCommand);

	}

	@Override
	public void teleopSequence() {

		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_Y_AXIS, driverController.LEFT_X_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.EPHEMERAL);
		driverController.A.whileHeld(new LockDrivetrainHeld(drivetrain), TaskPersistence.EPHEMERAL);
		driverController.B.whenPressed(
			new InstantCommand(
				() -> { drivetrain.zeroYaw(); }
			),
			TaskPersistence.EPHEMERAL
		);

		Pose2d shootingPoses = new Pose2d(1.30, 5.52, new Rotation2d(0));

		PathConstraints constraints = new PathConstraints(
			3.2, 4.0,
			Units.degreesToRadians(150), Units.degreesToRadians(150));


		Command pathfindingCommand = new PathfindHolonomic(
			shootingPoses,
			constraints,
			0.0,
			drivetrain::getPoseMeters,
			drivetrain::getRobotRelativeSpeeds,
			drivetrain::driveRobotRelative,
			holonomicPathFollowerConfig,
			0,
			drivetrain
		);

		driverController.LEFT_BUMPER.whileHeld(pathfindingCommand, TaskPersistence.GAMEPLAY);

	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
