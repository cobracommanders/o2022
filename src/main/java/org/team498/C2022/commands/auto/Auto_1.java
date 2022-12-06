package org.team498.C2022.commands.auto;

import org.team498.C2022.auto.PathLib;
import org.team498.C2022.commands.drivetrain.WPIDrive;
import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto_1 extends SequentialCommandGroup {
    public Auto_1(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            new WPIDrive(PathLib.auto_1_leg_1),
            new WPIDrive(PathLib.auto_1_leg_2)
        );
    }
}
