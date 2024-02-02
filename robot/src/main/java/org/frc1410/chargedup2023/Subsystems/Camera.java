package org.frc1410.chargedup2023.Subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

import static org.frc1410.chargedup2023.util.Constants.*;

public class Camera implements TickedSubsystem {

	private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

	private AprilTagFieldLayout layout;


	public Camera() {
		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
			var alliance = DriverStation.getAlliance();
//             layout.setOrigin(alliance == Optional.of(Alliance.Blue) ?
//                   OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
		} catch(IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			layout = null;
		}
	}

	 public PhotonPipelineResult getLatestResult() {
		return camera.getLatestResult();
	 }

	 public AprilTagFieldLayout aprilTagFieldLayout() {
		return layout;
	 }

	@Override
	public void periodic() {

	}
}