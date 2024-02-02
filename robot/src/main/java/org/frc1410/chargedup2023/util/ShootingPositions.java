package org.frc1410.chargedup2023.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootingPositions {

	public final Pose2d pose2d;
	public final double shooterRMP;

	public ShootingPositions(Pose2d pose2d, double shooterRPM) {
		this.pose2d = pose2d;
		this.shooterRMP = shooterRPM;
	}
}
