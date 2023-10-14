package org.frc1410.chargedup2023.Commands;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveTime extends ParallelRaceGroup {
    public DriveTime(Drivetrain drivetrain, double seconds) {
        this.addCommands(
            new InstantCommand(
                () -> drivetrain.drive(-2, 0, 0, false)
            ),
            new WaitCommand(seconds)
        );
    }

    // @Override
    // public void execute() {
    //     this.drivetrain.drive(-2, 0, 0, false);
    // }

    // @Override
    // public boolean isFinished() {
    //     return false;
    //     // return this.drivetrain.getPoseMeters().getY() >= this.distanceMeters;
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     this.drivetrain.drive(0, 0, 0, false);
    // }
}
