package org.team498.C2022.commands.auto;

import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto1 extends SequentialCommandGroup {
    public Pose2d startPose = new Pose2d(new Translation2d(8, 4), Rotation2d.fromDegrees(135));
    public Auto1(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            new InstantCommand(()-> drivetrain.offset = -startPose.getRotation().getDegrees()),
            new InstantCommand(()-> drivetrain.lastAngle = startPose.getRotation().getDegrees()),
            new InstantCommand(()-> drivetrain.setGyroOffset(0), drivetrain),
            new InstantCommand(()-> drivetrain.resetOdometry(startPose), drivetrain)
            //new SetPosition(drivetrain, 8, 4, -135)
           // new SetPosition(drivetrain, 6, 2, 45),
            //new SetPosition(drivetrain, 8, 8, -90)
        );
    }
}
