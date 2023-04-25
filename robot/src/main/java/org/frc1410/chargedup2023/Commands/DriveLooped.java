package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;


public class DriveLooped extends CommandBase {
	private final Drivetrain drivetrain;
	private final Axis rightVerticalAxis;
	private final Axis rightHorizontalAxis;
	private final boolean isFieldRelative;


	public DriveLooped(Drivetrain drivetrain, Axis rightVerticalAxis, Axis rightHorizontalAxis, boolean isFieldRelative) {
		this.drivetrain = drivetrain;
		this.rightVerticalAxis = rightVerticalAxis;
		this.rightHorizontalAxis = rightHorizontalAxis;
		this.isFieldRelative = isFieldRelative;

	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		drivetrain.drive(
			rightVerticalAxis.get(),
			rightHorizontalAxis.get(),
			0, true, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.setBreakMode();
	}
}
