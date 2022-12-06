package org.team498.C2022.commands.drivetrain;

import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPosition extends CommandBase {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final double x;
    private final double y;
	private final double angle;
	/**
	 * Snaps drivetrain to angle
	 * @requires {@link Drivetrain drivetrain}
	 * @param angle degrees
	 */
	public SetPosition(double x, double y, double angle) {
		this.angle = angle;
        this.x = x;
        this.y = y;
        addRequirements(drivetrain);
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
