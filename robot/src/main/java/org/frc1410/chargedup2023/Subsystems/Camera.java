package org.frc1410.chargedup2023.Subsystems;

import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera implements TickedSubsystem {

	private final PhotonCamera camera = new PhotonCamera("Camera 1");

	public PhotonPipelineResult getLatestrResult() {
		var lastResult = camera.getLatestResult();
		return lastResult;
	}

	public boolean hasTarget() {
		return getLatestrResult().hasTargets();
	}
	@Override
	public void periodic() {
		if(hasTarget()) {
			System.out.println("Found Target");
		} else {
			System.out.println("No target found");
		}
	}
}
