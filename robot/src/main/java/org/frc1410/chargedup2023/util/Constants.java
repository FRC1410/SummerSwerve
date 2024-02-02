package org.frc1410.chargedup2023.util;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

public interface Constants {
    // Controller constants
    int DRIVER_CONTROLLER = 0;
    int OPERATOR_CONTROLLER = 1;

    // Swerve module constants
    double DRIVING_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    double DRIVE_ROTATIONS_TO_METERES = (1.0 / DRIVING_GEAR_RATIO) * (2 * 0.0508 * Math.PI);

//    double MAX_ANGULAR_VEL = 25.57;
//    double MAX_ANGULAR_ACC = 100;  // radians per sec squared
    double MAX_SPEED = 4.2;
    // double MAX_ANGULAR_SPEED = Math.PI;

    double DRIVE_MOTOR_FREE_SPEED_RPM = 5676;
    double DRIVE_MOTOR_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPM / 60;
    double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * 0.0508;

    double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND = ((DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_GEAR_RATIO);


    int WHEEL_RADIUS = 2;
    int SWERVE_MODULE_ENCODER_RES = 4095;

    double FRONT_LEFT_STEER_ENCODER_OFFSET = 0.173340;
    double FRONT_RIGHT_STEER_ENCODER_OFFSET = 0.497314;
    double BACK_LEFT_STEER_ENCODER_OFFSET = -0.469482;
    double BACK_RIGHT_STEER_ENCODER_OFFSET = -0.300049;

    Translation2d FRONT_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(0.263525, 0.263525);
    Translation2d FRONT_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(0.263525, -0.263525);
    Translation2d BACK_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(-0.263525, 0.263525);
    Translation2d BACK_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(-0.263525, -0.263525);

    double SPEAKER_APRIL_TAG_DISTANCE = 20.25;

    double RIGHT_ANGLE_TO_RAD = Math.toRadians(90);

	String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

    Transform3d cameraToRobot = new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI));

	HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(1, 0.0, 0.0), // Translation PID constants
        new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
        3, // Max module speed, in m/s
        0.372680629, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

	HolonomicPathFollowerConfig pathfindPathFollowerConfig = new HolonomicPathFollowerConfig(
		new PIDConstants(1.0,0.0, 0.5),
		new PIDConstants(1, 0.0, 0),
		3,
		0.37268062,
		new ReplanningConfig()
	);

	PathConstraints constraints = new PathConstraints(
			3.2, 4.0,
			Units.degreesToRadians(150), Units.degreesToRadians(150));

	List<ShootingPositions> shootingPoses = Arrays.asList(
		new ShootingPositions(new Pose2d(1.36, 5.52, Rotation2d.fromDegrees(0)), 0),
		new ShootingPositions(new Pose2d(2.26, 4.81, Rotation2d.fromDegrees(-22.6)), 0),
		new ShootingPositions(new Pose2d(1.88, 5, Rotation2d.fromDegrees(-19.6)), 0)
	);
}
