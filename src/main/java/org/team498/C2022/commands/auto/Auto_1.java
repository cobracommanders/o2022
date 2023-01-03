package org.team498.C2022.commands.auto;

import org.team498.C2022.auto.PathLib;
import org.team498.C2022.commands.drivetrain.WPIDrive;
import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto_1 extends SequentialCommandGroup {
	private Drivetrain drivetrain = Drivetrain.getInstance();
    public Auto_1() {

        addRequirements(drivetrain);
        addCommands(
			new InstantCommand(() -> drivetrain.setGyroOffset(PathLib.unnamed.getInitialPose().getRotation().getDegrees())),
			new InstantCommand(() -> drivetrain.resetOdometry(PathLib.unnamed.getInitialPose())),
            new WPIDrive(PathLib.unnamed)
        );
    }
}
