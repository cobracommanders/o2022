package org.team498.C2022.commands.drivetrain;

import org.team498.C2022.DriverController;
import org.team498.C2022.RobotState;
import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TargetLock extends CommandBase {
    private final Drivetrain drivetrain;
    private final RobotState robotState;
    private final DriverController controller;
    private Double t;
    public TargetLock(Drivetrain drivetrain, RobotState robotState, DriverController controller) {
        this.drivetrain = drivetrain;
        this.robotState = robotState;
        this.controller = controller;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        t = 0.0;
    }
    @Override
    public void execute() {
        Transform2d state = robotState.getRobotToTarget();
        drivetrain.lockOnCircle(Math.hypot(state.getX(), state.getY()), state, controller.getX().getAsDouble(), t);
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds());
        t = 0.0;
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
