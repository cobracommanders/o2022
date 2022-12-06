package org.team498.C2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.lib.util.Trajectories;
import org.team498.C2022.commands.drivetrain.FieldOrientedDrive;
import org.team498.C2022.commands.drivetrain.SnapDrive;
import org.team498.C2022.commands.drivetrain.TrajectoryFollower;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.lib.drivers.Xbox;

import static org.team498.C2022.Constants.OIConstants;

public class RobotContainer {
    private final Drivetrain drivetrain;
    private static final Xbox xbox = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);

    public RobotContainer() {
        drivetrain = Drivetrain.getInstance();
        drivetrain.setInitialPose(new Pose2d(8, 4, Rotation2d.fromDegrees(90)));
        xbox.setDeadzone(0.2);
        xbox.setTriggerThreshold(0.2);

        configureButtonBindings();

        drivetrain.setDefaultCommand(new SnapDrive(xbox::leftX, xbox::leftY, xbox::RAngle));
    }


    private void configureButtonBindings() {
        xbox.A().whenActive(new InstantCommand(drivetrain::zeroGyro));
        xbox.X().toggleWhenActive(new FieldOrientedDrive(xbox::leftX, xbox::leftY, xbox::rightX));
        xbox.start().whenActive(new TrajectoryFollower(Trajectories.getTrajectory("test")));
    }

}

