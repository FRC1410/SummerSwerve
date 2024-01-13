package org.frc1410.chargedup2023.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.chargedup2023.util.IDs.*;

import org.frc1410.chargedup2023.util.NetworkTables;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

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

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;

	private final AHRS gyro = new AHRS(SPI.Port.kMXP);

	// private final PIDController headingPidController = new PIDController(2, 0, 0);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			FRONT_LEFT_SWERVE_MODULE_LOCATION,
			FRONT_RIGHT_SWERVE_MODULE_LOCATION,
			BACK_LEFT_SWERVE_MODULE_LOCATION,
			BACK_RIGHT_SWERVE_MODULE_LOCATION);

	public final SwerveDriveOdometry odometry;

	public boolean isLocked = false;


	// private Rotation2d desiredHeading = new Rotation2d();

	public Drivetrain(SubsystemStore subsystems) {
		this.frontLeft = subsystems.track(new SwerveModule(FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_STEER_MOTOR,
				FRONT_LEFT_STEER_ENCODER, false, true, FRONT_LEFT_STEER_ENCODER_OFFSET, frontLeftDesiredVel, frontLeftDesiredAngle, frontLeftActualVel, frontLeftActualAngle));
		this.frontRight = subsystems.track(new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_STEER_MOTOR,
				FRONT_RIGHT_STEER_ENCODER, true, true, FRONT_RIGHT_STEER_ENCODER_OFFSET, frontRightDesiredVel, frontRightDesiredAngle, frontRightActualVel, frontRightActualAngle));
		this.backLeft = subsystems.track(new SwerveModule(BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_STEER_MOTOR,
				BACK_LEFT_STEER_ENCODER, false, true, BACK_LEFT_STEER_ENCODER_OFFSET, backLeftDesiredVel, backLeftDesiredAngle, backLeftActualVel, backLeftActualAngle));
		this.backRight = subsystems.track(new SwerveModule(BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_STEER_MOTOR,
				BACK_RIGHT_STEER_ENCODER, true, true, BACK_RIGHT_STEER_ENCODER_OFFSET, backRightDesiredVel, backRightDesiredAngle, backRightActualVel, backRightActualAngle));

		this.odometry = new SwerveDriveOdometry(
				kinematics,
				gyro.getRotation2d(),
				new SwerveModulePosition[] {
						frontLeft.getPosition(),
						frontRight.getPosition(),
						backLeft.getPosition(),
						backRight.getPosition()
				});

		// this.headingPidController.enableContinuousInput(-Math.PI, Math.PI);

		gyro.reset();

		// gyro.calibrate();

		// AutoBuilder.configureHolonomic(
        //         this::getPoseMeters, // Robot pose supplier
        //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        //                 4.5, // Max module speed, in m/s
        //                 0.372680629, // Drive base radius in meters. Distance from robot center to furthest module.
        //                 new ReplanningConfig() // Default path replanning config. See the API for the options here
        //         ),
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             // var alliance = DriverStation.getAlliance();
        //             // if (alliance.isPresent()) {
        //             //     return alliance.get() == DriverStation.Alliance.Red;
        //             // }
        //             return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        // );

	// 	// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
	// 	HashMap<String, Command> eventMap = new HashMap<>();

	// 	// PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
		
	
	// 	this.autoBuilder = new SwerveAutoBuilder(
	// 		this::getPoseMeters, // Pose2d supplier
	// 		this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
	// 		// this.kinematics, // SwerveDriveKinematics
	// 		new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
	// 		new PIDConstants(0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
	// 		this::driveRobotRelative, // Module states consumer used to output to the drive subsystem
	// 		eventMap,
	// 		false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
	// 		this // The drive subsystem. Used to properly set the requirements of path following commands
	// )	;

		AutoBuilder.configureHolonomic(
                this::getPoseMeters, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0, 0.0, 0.0), // Rotation PID constants
                        3, // Max module speed, in m/s
                        0.372680629, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Blue;
                    // }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
	}

	void setModuleStates(SwerveModuleState[] a) {
		// this.driveRobotRelative(a);;
	}

	// private float yawOffset;

	public void resetPose(Pose2d pose) {
		this.odometry.resetPosition(
			this.gyro.getRotation2d(),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				backLeft.getPosition(),
				backRight.getPosition()
			},
			pose);
	  }

	public void zeroYaw() {
		// System.out.println("zero Yaw");
		this.gyro.zeroYaw();
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

	// public void driveKeepingHeading(double xVelocity, double yVelocity, boolean isFieldRelative) {
	// 	var headingAdjustment = this.headingPidController.calculate(this.gyro.getRotation2d().getRadians(), this.desiredHeading.getRadians());
	// 	// navXYaw.set(-headingAdjustment);
	// 	this.driveWithRotation(xVelocity, yVelocity, -headingAdjustment, isFieldRelative);
	// }

	public void driveRobotRelative(ChassisSpeeds speeds){
		this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
	}

	public void drive(double xVelocity, double yVelocity, double rotation, boolean isFieldRelative) {
		if (isLocked) {
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

			// Pose2d robot_pose_vel = new Pose2d(
			// 	chassisSpeeds.vxMetersPerSecond * 0.02, 
			// 	chassisSpeeds.vyMetersPerSecond * 0.02, 
			// 	Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * 0.02)
			// );

			// Twist2d twist_vel = Drivetrain.log(robot_pose_vel);

			// ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
            // 	twist_vel.dx / 0.02,
			// 	twist_vel.dy / 0.02,
			// 	twist_vel.dtheta / 0.02
			// );

			// var swerveModuleStates = kinematics.toSwerveModuleStates(correctForDynamics(chassisSpeeds));
			var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

			SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

			frontLeft.setDesiredState(swerveModuleStates[1]);
			frontRight.setDesiredState(swerveModuleStates[3]);
			backLeft.setDesiredState(swerveModuleStates[0]);
			backRight.setDesiredState(swerveModuleStates[2]);
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
		return this.odometry.getPoseMeters();
	}

	public void updateOdometry() {
		odometry.update(
				gyro.getRotation2d(),
				new SwerveModulePosition[] {
						// frontLeft.getPosition(),
						// frontRight.getPosition(),
						// backLeft.getPosition(),
						// backRight.getPosition()
						backLeft.getPosition(),
						frontLeft.getPosition(),
						backRight.getPosition(),
						frontRight.getPosition()

				});
		
				// navXYaw.set(this.odometry.getPoseMeters().getX());
	}

	@Override
	public void periodic() {
		navXYaw.set(gyro.getPitch());
		updateOdometry();
	}
}
