package org.frc1410.chargedup2023.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.Subsystems.Camera;


public class CameraTestCommand extends Command {

	private final Camera camera;

	public CameraTestCommand(Camera camera) {
		this.camera = camera;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if(camera.hasTargets()) {
			camera.getTargetYaw();
//			camera.getTargetYaw(1);
			System.out.println("Target's found");

//			camera.getDistance();
//		} else if(camera.hasTargets()) {
//			camera.getTargetYaw(0);
//			System.out.println("Target found");
//		} else if(!camera.hasTargets()){
//			System.out.println("Target / targets not found");
//		} else {
//			System.out.println("Shits borked");
		} else if(!camera.hasTargets()) {

		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
//		System.out.println("Camera command shutdown");
	}
}
