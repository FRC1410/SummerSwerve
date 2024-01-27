package org.frc1410.chargedup2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.InstantCommand;

//import org.frc1410.chargedup2023.Subsystems.Camera;
import org.frc1410.chargedup2023.Commands.PathFindToNearestPose;
import org.frc1410.chargedup2023.Subsystems.Camera;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.AutoSelector;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import com.pathplanner.lib.commands.PathPlannerAuto;

import static org.frc1410.chargedup2023.util.Constants.*;

import org.frc1410.chargedup2023.Commands.DriveLooped;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

public final class Robot extends PhaseDrivenRobot {

	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.2);
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER, 0.2);

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));
	private final Camera camera = subsystems.track(new Camera());

	private final AutoSelector autoSelector = new AutoSelector()
		.add("PR-B#-A# (3pS)", () -> new PathPlannerAuto("PR-B#-A# (3pS)"))
		.add("PR-B#-C# (3pS)", () -> new PathPlannerAuto("PR-B#-C# (3pS)"))
		.add("PR-A# (1pA)", () -> new PathPlannerAuto("PR-A# (1pA)"));

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

	@Override
	public void autonomousSequence() {

		 NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
		 String autoProfile = autoSubscriber.get();
		 var autoCommand = autoSelector.select(autoProfile);

		scheduler.scheduleAutoCommand(autoCommand);
	}

	@Override
	public void teleopSequence() {

		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_Y_AXIS, driverController.LEFT_X_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.EPHEMERAL);
		// driverController.A.whileHeld(new LockDrivetrainHeld(drivetrain), TaskPersistence.EPHEMERAL);
		driverController.B.whenPressed(
			new InstantCommand(
				() -> { drivetrain.zeroYaw(); }
			),
			TaskPersistence.EPHEMERAL
		);

		driverController.RIGHT_BUMPER.whileHeldOnce(new PathFindToNearestPose(drivetrain), TaskPersistence.GAMEPLAY);
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
