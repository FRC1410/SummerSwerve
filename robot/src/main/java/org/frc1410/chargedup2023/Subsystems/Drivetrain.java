package org.frc1410.chargedup2023.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import java.util.Optional;

import org.frc1410.chargedup2023.util.NetworkTables;

import static org.frc1410.chargedup2023.util.IDs.*;
import static org.frc1410.chargedup2023.util.Constants.*;

public class Drivetrain implements TickedSubsystem {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

    private final DoublePublisher chassisSpeedsX = NetworkTables.PublisherFactory(table, "ChassisSpeeds X", 0);
    private final DoublePublisher chassisSpeedsY = NetworkTables.PublisherFactory(table, "ChassisSpeeds Y", 0);
    private final DoublePublisher chassisSpeedsRotation = NetworkTables.PublisherFactory(table,
            "ChassisSpeeds Rotation", 0);

    private final DoublePublisher frontLeftVoltage = NetworkTables.PublisherFactory(table, "frontLeftVoltage", 0);
    private final DoublePublisher frontRightVoltage = NetworkTables.PublisherFactory(table, "frontRightVoltage", 0);
    private final DoublePublisher backLeftVoltage = NetworkTables.PublisherFactory(table, "backLeftVoltage", 0);
    private final DoublePublisher backRightVoltage = NetworkTables.PublisherFactory(table, "backRightVoltage", 0);

    private final DoublePublisher navXYaw = NetworkTables.PublisherFactory(table, "NavX Yaw", 0);

    private final DoublePublisher frontLeftDesiredVel = NetworkTables.PublisherFactory(table, "frontLeft Desired Vel", 0);
    private final DoublePublisher frontRightDesiredVel = NetworkTables.PublisherFactory(table, "frontRight Desired Vel", 0);
    private final DoublePublisher backLeftDesiredVel = NetworkTables.PublisherFactory(table, "backLeft Desired Vel", 0);
    private final DoublePublisher backRightDesiredVel = NetworkTables.PublisherFactory(table, "backRight Desired Vel", 0);

    private final DoublePublisher frontLeftDesiredAngle = NetworkTables.PublisherFactory(table, "frontLeft Desired angle", 0);
    private final DoublePublisher frontRightDesiredAngle = NetworkTables.PublisherFactory(table, "frontRight Desired angle", 0);
    private final DoublePublisher backLeftDesiredAngle = NetworkTables.PublisherFactory(table, "backLeft Desired angle", 0);
    private final DoublePublisher backRightDesiredAngle = NetworkTables.PublisherFactory(table, "backRight Desired angle", 0);

    private final DoublePublisher frontLeftActualVel = NetworkTables.PublisherFactory(table, "frontLeft Actual Vel", 0);
    private final DoublePublisher frontRightActualVel = NetworkTables.PublisherFactory(table, "frontRight Actual Vel", 0);
    private final DoublePublisher backLeftActualVel = NetworkTables.PublisherFactory(table, "backLeft Actual Vel", 0);
    private final DoublePublisher backRightActualVel = NetworkTables.PublisherFactory(table, "backRight Actual Vel", 0);

    private final DoublePublisher frontLeftActualAngle = NetworkTables.PublisherFactory(table, "frontLeft Actual angle", 0);
    private final DoublePublisher frontRightActualAngle = NetworkTables.PublisherFactory(table, "frontRight Actual angle", 0);
    private final DoublePublisher backLeftActualAngle = NetworkTables.PublisherFactory(table, "backLeft Actual angle", 0);
    private final DoublePublisher backRightActualAngle = NetworkTables.PublisherFactory(table, "backRight Actual angle", 0);

    private final DoublePublisher poseX = NetworkTables.PublisherFactory(table, "estimated pose X", 0);
    private final DoublePublisher poseY = NetworkTables.PublisherFactory(table, "estimated pose Y", 0);
    // private final DoublePublisher poseZ = NetworkTables.PublisherFactory(table, "estimated pose Z", 0);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private double previousPipelineTimestamp = 0;

	private final Camera camera;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            FRONT_LEFT_SWERVE_MODULE_LOCATION,
            FRONT_RIGHT_SWERVE_MODULE_LOCATION,
            BACK_LEFT_SWERVE_MODULE_LOCATION,
            BACK_RIGHT_SWERVE_MODULE_LOCATION);

    private final SwerveDrivePoseEstimator poseEstimator;

    public boolean isLocked = false;


    public Drivetrain(SubsystemStore subsystems, Camera camera) {
        this.frontLeft = subsystems.track(new SwerveModule(FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_STEER_MOTOR,
                FRONT_LEFT_STEER_ENCODER, true, true, FRONT_LEFT_STEER_ENCODER_OFFSET, frontLeftDesiredVel, frontLeftDesiredAngle, frontLeftActualVel, frontLeftActualAngle));
        this.frontRight = subsystems.track(new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_STEER_MOTOR,
                FRONT_RIGHT_STEER_ENCODER, false, true, FRONT_RIGHT_STEER_ENCODER_OFFSET, frontRightDesiredVel, frontRightDesiredAngle, frontRightActualVel, frontRightActualAngle));
        this.backLeft = subsystems.track(new SwerveModule(BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_STEER_MOTOR,
                BACK_LEFT_STEER_ENCODER, true, true, BACK_LEFT_STEER_ENCODER_OFFSET, backLeftDesiredVel, backLeftDesiredAngle, backLeftActualVel, backLeftActualAngle));
        this.backRight = subsystems.track(new SwerveModule(BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_STEER_MOTOR,
                BACK_RIGHT_STEER_ENCODER, false, true, BACK_RIGHT_STEER_ENCODER_OFFSET, backRightDesiredVel, backRightDesiredAngle, backRightActualVel, backRightActualAngle));

        this.poseEstimator = new SwerveDrivePoseEstimator(
            this.kinematics,
            this.gyro.getRotation2d(),
            this.getSwerveModulePositions(),
            new Pose2d()
        );

		this.camera = camera;

        gyro.reset();

        AutoBuilder.configureHolonomic(
               this::getPoseMeters,
               this::resetPose,
               this::getRobotRelativeSpeeds,
               this::driveRobotRelative,
               holonomicPathFollowerConfig,
               () -> {

                   // var alliance = DriverStation.getAlliance();
                   // if (alliance.isPresent()) {
                   //     return alliance.get() == DriverStation.Alliance.Blue;
                   // }
                   return false;
               },
               this
       );

    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            };
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPosition(
            this.gyro.getRotation2d(),
            this.getSwerveModulePositions(),
            pose
        );
    }

    public void zeroYaw() {
        // System.out.println("zero Yaw");
        this.gyro.zeroYaw();
		this.resetPose(new Pose2d(this.getPoseMeters().getTranslation(), this.gyro.getRotation2d()));
        // this.desiredHeading = this.gyro.getRotation2d();
        // yawOffset = this.gyro.getRotation2d()
    }

    public void zero() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(this.gyro.getPitch());
    }

    public static Twist2d log(final Pose2d transform) {

        double kEps = 1E-9;
       final double dtheta = transform.getRotation().getRadians();
       final double half_dtheta = 0.5 * dtheta;
       final double cos_minus_one = transform.getRotation().getCos() - 1.0;
       double halftheta_by_tan_of_halfdtheta;
       if (Math.abs(cos_minus_one) < kEps) {
           halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
       } else {
           halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
       }
       final Translation2d translation_part = transform.getTranslation()
               .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
       return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
   }

    private ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose = new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = log(futureRobotPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    public static ChassisSpeeds discretize(double vx, double vy, double omega, double dt) {
        var desiredDeltaPose = new Pose2d(vx * dt, vy * dt, new Rotation2d(omega * dt));
        var twist = new Pose2d().log(desiredDeltaPose);
        return new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt);
    }

    public static ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dt) {
        return discretize(continuousSpeeds.vxMetersPerSecond, continuousSpeeds.vyMetersPerSecond, continuousSpeeds.omegaRadiansPerSecond, dt);
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
		System.out.println("drive robot relative " + speeds);
        this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    public void drive(double xVelocity, double yVelocity, double rotation, boolean isFieldRelative) {
        if (isLocked) {
			System.out.println("LOCKED");
            frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
            backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
            backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        } else {
            chassisSpeedsX.set(xVelocity);
            chassisSpeedsY.set(yVelocity);
            chassisSpeedsRotation.set(rotation);

            var chassisSpeeds = isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotation, gyro.getRotation2d())
                : new ChassisSpeeds(xVelocity, yVelocity, rotation);

            var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

            frontLeft.setDesiredState(swerveModuleStates[0]);
            frontRight.setDesiredState(swerveModuleStates[1]);
            backLeft.setDesiredState(swerveModuleStates[2]);
            backRight.setDesiredState(swerveModuleStates[3]);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
        	frontLeft.getState(),
           	frontRight.getState(),
           	backLeft.getState(),
           	backRight.getState()
        );
    }

    public Pose2d getPoseMeters() {
        // return this.odometry.getPoseMeters();
        return this.poseEstimator.getEstimatedPosition();
    }

    public void updateOdometry() {
        this.poseEstimator.update(
            this.gyro.getRotation2d(),
            this.getSwerveModulePositions()
        );
    }

    @Override
    public void periodic() {
        navXYaw.set(gyro.getPitch());
        updateOdometry();

		var estimatedPose = camera.getEstimatedPose();

		if(estimatedPose.isPresent()) {

			// TODO: Possible bug where bad data is fed into pose estimator when no vision
			var resultTimestamp = estimatedPose.get().timestampSeconds;

			if(resultTimestamp != previousPipelineTimestamp) {
				previousPipelineTimestamp = resultTimestamp;
				poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), resultTimestamp);
			}
		}

        System.out.println(this.getPoseMeters());
        var pose = this.getPoseMeters();
        this.poseX.set(pose.getX());
        this.poseY.set(pose.getY());
    }
}
