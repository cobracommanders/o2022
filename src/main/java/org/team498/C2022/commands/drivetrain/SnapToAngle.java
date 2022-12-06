package org.team498.C2022.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2022.subsystems.Drivetrain;

public class SnapToAngle extends CommandBase {
    private final Drivetrain drivetrain;
    private final double angle;

    public SnapToAngle(double angle) {
        this.drivetrain = Drivetrain.getInstance();
        this.angle = angle;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setSnapGoal(angle);
    }

    @Override
    public void execute() {
        // Calculate the rotational speed from the pid controller, unless it's already at the goal
        double rotationalSpeed = drivetrain.calculateSnapSpeed();

        // Set the drivetrain to drive, remaining in the set position but snapping to the target
        drivetrain.drive(new ChassisSpeeds(0, 0, rotationalSpeed));
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atSnapGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }


}
