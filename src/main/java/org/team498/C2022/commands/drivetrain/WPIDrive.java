package org.team498.C2022.commands.drivetrain;

import org.team498.C2022.subsystems.Drivetrain;
import org.team498.lib.math.trajectory.Trajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WPIDrive extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Trajectory trajectory;
    Timer timer = new Timer();
    /**
     * requires {@link Drivetrain drivetrain}
     * @param trajectory {@link Trajectory} to follow
     */
    public WPIDrive(Trajectory trajectory) {
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
        ChassisSpeeds adjustedSpeeds = drivetrain.driveController.calculate(drivetrain.getPose(), goal, goal.poseMeters.getRotation());
        drivetrain.drive(adjustedSpeeds);
    }
    @Override
    public boolean isFinished() {
        return drivetrain.driveController.atReference();
    }
}
