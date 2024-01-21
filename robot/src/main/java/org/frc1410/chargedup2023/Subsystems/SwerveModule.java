package org.frc1410.chargedup2023.Subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;

import static org.frc1410.chargedup2023.util.Constants.*;
import static org.frc1410.chargedup2023.util.Tuning.*;

import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class SwerveModule implements TickedSubsystem {

	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final CANcoder steerEncoder;

	// private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);

	private final SparkPIDController drivePIDController;

	private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV);

	// private final PIDController turningPIDController = new PIDController(SWERVE_STEERING_KP, SWERVE_STEERING_KI, SWERVE_STEERING_KD);
	private final PIDController turningPIDController = new PIDController(
		SWERVE_STEERING_KP,
		SWERVE_STEERING_KI,
		SWERVE_STEERING_KD
		// 27 rad/s
		// 135 rad/s/s
		// new TrapezoidProfile.Constraints(Math.PI, 2 * Math.PI)
		// new TrapezoidProfile.Constraints(27, 135)
	);
	// private final SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(0.15, 0.5);

	public SwerveModuleState desiredState = new SwerveModuleState();


	private final DoublePublisher desiredVel;
	private final DoublePublisher desiredAngle;

	private final DoublePublisher actualVel;
	private final DoublePublisher actualAngle;

	public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, boolean driveMotorInverted, boolean steerMotorInverted, double offset, DoublePublisher desiredVel, DoublePublisher desiredAngle, DoublePublisher actualVel, DoublePublisher actualAngle) {


		driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		driveMotor.setInverted(driveMotorInverted);
		driveMotor.setSmartCurrentLimit(40);

		this.drivePIDController = driveMotor.getPIDController();
		this.drivePIDController.setP(SWERVE_DRIVE_KP);
		this.drivePIDController.setI(SWERVE_DRIVE_KI);
		this.drivePIDController.setD(SWERVE_DRIVE_KD);
		// this.drivePIDController.setIZone();
		// this.drivePIDController.setFF(0.000015);
		this.drivePIDController.setFF(SWERVE_DRIVE_KFF);
		// this.drivePIDController.setOutputRange(-1, 1);
		// drivePIDController1.
		// CANSparkLowLevel.MotorType.kBrushless
		steerMotor = new CANSparkMax(steeringMotorID, CANSparkLowLevel.MotorType.kBrushless);
		steerMotor.restoreFactoryDefaults();
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steerMotor.setInverted(steerMotorInverted);
		steerMotor.setSmartCurrentLimit(30);

		driveEncoder = driveMotor.getEncoder();

		steerEncoder = new CANcoder(steeringEncoderID);
		// steerEncoder.configMagnetOffset(-offset);
		CANcoderConfigurator configurator = steerEncoder.getConfigurator();
		var config = new CANcoderConfiguration();
		config.MagnetSensor.MagnetOffset = offset;

		configurator.apply(config);



		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		this.desiredVel = desiredVel;
		this.desiredAngle = desiredAngle;

		this.actualVel = actualVel;
		this.actualAngle = actualAngle;

		// ?
		// this.turningPIDController.reset(getSteerPosition());
	}

	@Override
	public void periodic() {
		// double driveFeedOutput = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
		// double drivePIDOutput = drivePIDController.calculate(getDriveVelocityMetersPerSecond(), desiredState.speedMetersPerSecond);



		// System.out.println("" + getDriveVelocityMetersPerSecond() + " | " + desiredState.speedMetersPerSecond);

		double steerPIDOutput = turningPIDController.calculate(getSteerPosition(), MathUtil.angleModulus(desiredState.angle.getRadians()));
		// double turnFeedOutput = turningFeedforward.calculate(turningPIDController.getSetpoint().velocity);

		// this.actualVel.set(getDriveVelocityMetersPerSecond());
		// this.desiredVel.set(desiredState.speedMetersPerSecond);
		this.actualAngle.set(new Rotation2d(this.getSteerPosition()).getRotations());

		// driveMotor.setVoltage(drivePIDOutput);



		steerMotor.setVoltage(steerPIDOutput);




		// steerMotor.setVoltage(12);
		// voltage.set(turnFeedOutput);
		// voltage.set(getSteerPosition());


		// voltage.set(Units.radiansToDegrees(getSteerPosition()));
		// this.steerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(steerEncoder.getPosition().getValue()));
		this.desiredState = optimized;



		this.drivePIDController.setReference(metersPerSecondToEncoderRPM(optimized.speedMetersPerSecond), CANSparkMax.ControlType.kVelocity);
		// this.actualVel.set(Math.abs(driveEncoder.getVelocity()));
		// this.actualVel.set(this.driveMotor.get);
		this.desiredVel.set(Math.abs(metersPerSecondToEncoderRPM(optimized.speedMetersPerSecond)));


		// this.desiredAngle.set(optimized.angle.getDegrees());
	}

	public double metersPerSecondToEncoderRPM(double metersPerSecond) {
		return ((metersPerSecond * 60) / WHEEL_CIRCUMFERENCE_METERS) * DRIVING_GEAR_RATIO;
		// return (metersPerSecond * 60) / DRIVE_ROTATIONS_TO_METERES;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			getDrivePositionMeters(), new Rotation2d(getSteerPosition())
		);
	}

	public double getDriveVelocityMetersPerSecond() {
		return (driveEncoder.getVelocity() * DRIVE_ROTATIONS_TO_METERES) / 60;
	}

	public double getDrivePositionMeters() {
		return driveEncoder.getPosition() * DRIVE_ROTATIONS_TO_METERES;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(this.getDriveVelocityMetersPerSecond(), Rotation2d.fromRadians(getSteerPosition()));
	}

	public double getSteerPosition() {
		double rem = (steerEncoder.getPosition().getValue() * 360) % 360;

		double res;

		if (rem > 180) {
			res = rem - 360;
		} else if (rem <= -180) {
			res = rem + 360;
		} else {
			res = rem;
		}

		return Units.degreesToRadians(res);
	}
}