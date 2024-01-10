package org.frc1410.chargedup2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.AutoSelector;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import static org.frc1410.chargedup2023.util.Constants.*;

import java.util.HashMap;

import org.frc1410.chargedup2023.Commands.DriveLooped;
import org.frc1410.chargedup2023.Commands.LockDrivetrainHeld;
import org.frc1410.chargedup2023.Commands.Auto.Engage;
import org.frc1410.chargedup2023.Commands.Auto.Forward;
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
		.add("ENGAGE", () -> new Engage(this.drivetrain))
		.add("FORWARD", () -> new Forward(this.drivetrain));
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
		HashMap<String, Command> eventMap = new HashMap<>();

		// PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
		PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
	
		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
			this.drivetrain::getPoseMeters, // Pose2d supplier
			this.drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
			// this.kinematics, // SwerveDriveKinematics
			new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			this.drivetrain::driveRobotRelative, // Module states consumer used to output to the drive subsystem
			eventMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			this.drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
	)	;


		Command fullAuto = autoBuilder.fullAuto(examplePath);


		// NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
		// String autoProfile = autoSubscriber.get();
		// var autoCommand = autoSelector.select(autoProfile);
		scheduler.scheduleAutoCommand(fullAuto);
	}

	@Override
	public void teleopSequence() {
		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_Y_AXIS, driverController.LEFT_X_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.GAMEPLAY);
		driverController.A.whileHeld(new LockDrivetrainHeld(drivetrain), TaskPersistence.EPHEMERAL);
		driverController.B.whenPressed(
			new InstantCommand(
				() -> { drivetrain.zeroYaw(); }
			),
			TaskPersistence.EPHEMERAL
		);
		// drivetrain.zero();
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
