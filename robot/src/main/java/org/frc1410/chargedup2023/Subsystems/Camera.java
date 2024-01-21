package org.frc1410.chargedup2023.Subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class Camera implements Subsystem {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Vision Data");
//	DoublePublisher distancePub = NetworkTables.PublisherFactory(table, "Distance from target", 0);
//	DoublePublisher aprilYawPub = NetworkTables.PublisherFactory(table, "Camera Yaw" , 0);
//	DoublePublisher aprilYawPubTwo = NetworkTables.PublisherFactory(table, "Second Camera Yaw", 0);


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

	public double getTargetYaw() {

		var result = camera.getLatestResult();
//		List<PhotonTrackedTarget> targets = result.getTargets();
//
//		PhotonTrackedTarget selectedTarget = targets.get(targetIndex);
		PhotonTrackedTarget selectedTarget = result.getBestTarget();
		return selectedTarget.getYaw();

	}


//	public double getDistance() {
//
//		double hypotenuseOne =
//			(Math.sin(RIGHT_ANGLE_TO_RAD - Math.toRadians(getTargetYaw(1))))
//				* (SPEAKER_APRIL_TAG_DISTANCE / Math.sin(Math.toRadians(getTargetYaw(0) + getTargetYaw(1))));
//
//		double distanceFromTarget = (RIGHT_ANGLE_TO_RAD - getTargetYaw(0)) * (hypotenuseOne / Math.sin(RIGHT_ANGLE_TO_RAD));
//		return distanceFromTarget;
//	}

//	public Pose2d getRobotCameraPose() {
//		double xCordinate = Math.sin(getTargetYaw(0) * (getDistance() / (90 - getTargetYaw(0))));
//
//		return new Pose2d(xCordinate, getDistance(), new Rotation2d(0));
//	}
	@Override
	public void periodic() {

//		distancePub.set(getDistance());
//		aprilYawPub.set(getTargetYaw());
//		aprilYawPubTwo.set(getTargetYaw(1));
	}
}