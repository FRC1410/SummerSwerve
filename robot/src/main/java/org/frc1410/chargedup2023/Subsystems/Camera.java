package org.frc1410.chargedup2023.Subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import static org.frc1410.chargedup2023.util.Constants.*;

import java.io.IOException;
import java.util.List;

public class Camera implements TickedSubsystem {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Vision Data");
	DoublePublisher distancePub = NetworkTables.PublisherFactory(table, "Distance from target", 0);
	DoublePublisher aprilYawPub = NetworkTables.PublisherFactory(table, "Camera Yaw" , 0);
	DoublePublisher aprilYawPubTwo = NetworkTables.PublisherFactory(table, "Second Camera Yaw", 0);


	private final PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

//	private static final AprilTagFieldLayout fieldLayout;
//
//    static {
//        try {
//            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//    }

	public boolean hasTargets() {
		return camera.getLatestResult().hasTargets();
		// camera.getLatestResult().getMultiTagResult()
	}

	public int getlength() {
		List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
		return targets.size();
	}

	public double getTargetYaw(int targetIndex) {

		var result = camera.getLatestResult();
		List<PhotonTrackedTarget> targets = result.getTargets();

		PhotonTrackedTarget selectedTarget = targets.get(targetIndex);
//		PhotonTrackedTarget selectedTarget = result.getBestTarget();
		return selectedTarget.getYaw();
	}

	public Transform3d getCameraToApril() {
		var result = camera.getLatestResult();

		return result.getBestTarget().getBestCameraToTarget();
	}

	@Override
	public void periodic() {

	}
}