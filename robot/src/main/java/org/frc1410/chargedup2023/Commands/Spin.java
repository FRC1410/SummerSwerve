package org.frc1410.chargedup2023.Commands;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class Spin extends Command {
    private final Drivetrain drivetrain;

    public Spin(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        this.drivetrain.drive(0, 0, 1, false);
    }

    @Override
    public boolean isFinished() {
        return false;
        // return this.drivetrain.getPoseMeters().getY() >= this.distanceMeters;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.drive(0, 0, 0, false);
    }
}
