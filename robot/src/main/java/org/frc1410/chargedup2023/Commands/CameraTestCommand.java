package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.Subsystems.Camera;


public class CameraTestCommand extends Command {

	private final Camera camera;

	public CameraTestCommand(Camera camera) {
		this.camera = camera;
		addRequirements();
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if(camera.hasTargets()) {
			camera.getTargetYaw(0);
			camera.getTargetYaw(1);

			camera.getDistance();
		} else if(camera.hasTarget()) {
			camera.getTargetYaw(0);
		} else {
			System.out.println("Target / targets not found");
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("Camera command shutdown");
	}
}
