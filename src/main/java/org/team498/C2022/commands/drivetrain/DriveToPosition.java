package org.team498.C2022.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2022.subsystems.Drivetrain;

public class DriveToPosition extends CommandBase {
    private final Drivetrain drivetrain;
    private final double xGoal;
    private final double yGoal;
    private final double angle;

    public DriveToPosition(double xGoal, double yGoal, double angle) {
        this.drivetrain = Drivetrain.getInstance();
        this.angle = angle;
        this.xGoal = xGoal;
        this.yGoal = yGoal;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPositionGoals(xGoal, yGoal, angle);
    }

    @Override
    public void execute() {
        drivetrain.driveToPositionGoals();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atPositionGoals();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
