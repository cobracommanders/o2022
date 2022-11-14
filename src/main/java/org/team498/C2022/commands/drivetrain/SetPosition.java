package org.team498.C2022.commands.drivetrain;

import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPosition extends CommandBase {
	private final Drivetrain drivetrain;
    private final double x;
    private final double y;
	private final double angle;
	/**
	 * Snaps drivetrain to angle
	 * @param drivetrain
	 * @param angle degrees
	 */
	public SetPosition(Drivetrain drivetrain, double x, double y, double angle) {
		this.drivetrain = drivetrain;
		this.angle = angle;
        this.x = x;
        this.y = y;
        addRequirements(drivetrain);
        drivetrain.setPosition(x, y, angle);
	}

	@Override
	public void execute() {
        drivetrain.setPosition(x, y, angle);
	}
	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds());
	}
	@Override
	public boolean isFinished() {
		return drivetrain.atPosition();
	}
}
