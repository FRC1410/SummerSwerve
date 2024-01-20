package org.frc1410.chargedup2023.Commands;

import org.frc1410.chargedup2023.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PIDTestCommnd extends SequentialCommandGroup {
    public PIDTestCommnd(Drivetrain drivetrain) {
        this.addCommands(
            new ParallelRaceGroup(
                new Spin(drivetrain)
            )
        );
    }
}
