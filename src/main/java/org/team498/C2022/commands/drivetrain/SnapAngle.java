package org.team498.C2022.commands.drivetrain;

import org.team498.C2022.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SnapAngle extends CommandBase {
	private final Drivetrain drivetrain;
	private final double angle;
	/**
	 * Snaps drivetrain to angle
	 * @param drivetrain
	 * @param angle degrees
	 */
	public SnapAngle(Drivetrain drivetrain, double angle) {
		this.drivetrain = drivetrain;
		this.angle = angle;

	}

	@Override
	public void execute() {
		drivetrain.startSnap(angle);
	}
	@Override
	public void end(boolean interrupted) {
		drivetrain.maybeStopSnap(true);
	}
	@Override
	public boolean isFinished() {
		return drivetrain.snapComplete();
	}
}
