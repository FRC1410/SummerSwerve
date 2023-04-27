package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;


public class LockDrivetrain extends CommandBase {

	private final Drivetrain drivetrain;
	private final boolean isLocked;
	public LockDrivetrain(Drivetrain drivetrain, boolean isLocked) {
		this.drivetrain = drivetrain;
		this.isLocked = isLocked;
	}

	@Override
	public void initialize() {
		drivetrain.drive(0,0,0,true, isLocked);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.setBreakMode();
	}
}
