package org.team498.C2022.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2022.subsystems.Drivetrain;

public class TrajectoryFollower extends CommandBase {
    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    public TrajectoryFollower(Trajectory trajectory) {
        this.drivetrain = Drivetrain.getInstance();
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());

        ChassisSpeeds speeds = drivetrain.getSpeedsFromTrajectoryState(goal);
        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atTrajectoryGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
