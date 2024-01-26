package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.Subsystems.Camera;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;


public class DriveLooped extends Command {
	private final Drivetrain drivetrain;

	private final Axis xAxis;
	private final Axis yAxis;

	private final Axis rotationAxis;

	private boolean previousTickHadInput = false;
	
	public DriveLooped(Drivetrain drivetrain, Axis xAxis, Axis yAxis, Axis rotationAxis) {
		this.drivetrain = drivetrain;

		this.xAxis = xAxis;
		this.yAxis = yAxis;
		this.rotationAxis = rotationAxis;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		var xVelocity = xAxis.get() * 4;
		var yVelocity = yAxis.get() * 4;
		var rotation = rotationAxis.get() * 4;

		var hasInput = xVelocity != 0 || yVelocity != 0 || rotation != 0;

		if (!previousTickHadInput && hasInput) {
			drivetrain.isLocked = false;
		}

		System.out.println("does have input");
		drivetrain.drive(xVelocity, yVelocity, rotation, true);

		previousTickHadInput = hasInput;
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
