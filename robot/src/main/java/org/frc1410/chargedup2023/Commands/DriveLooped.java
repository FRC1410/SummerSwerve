package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;
import org.frc1410.framework.control.Axis;


public class DriveLooped extends CommandBase {
	private final Drivetrain drivetrain;
	private final Axis rightYAxis;
	private final Axis rightXAxis;
	private final Axis leftXAxis;


	public DriveLooped(Drivetrain drivetrain, Axis rightYAxis, Axis rightXAxis, Axis leftXAxis) {
		this.drivetrain = drivetrain;
		this.rightYAxis = rightYAxis;
		this.rightXAxis = rightXAxis;
		this.leftXAxis = leftXAxis;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		drivetrain.drive(
			rightYAxis.get(),
			rightXAxis.get(),
			leftXAxis.get(),
			true, false);
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
