package org.team498.C2022.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team498.C2022.DriverController;
import org.team498.C2022.subsystems.Drivetrain;

// Controls the robot in fiels oriented mode
public class TeleDrive extends CommandBase {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final DriverController controller = DriverController.getInstance();

	private final DoubleSupplier xSupplier;
	private final DoubleSupplier ySupplier;
	private final DoubleSupplier offenseRotationSupplier;
	private final DoubleSupplier defenseRotationSupplier;
	private final BooleanSupplier offense;
	private final DoubleSupplier driveSpeed;

	/**
	 * requires {@link Drivetrain drivetrain} and uses {@link DriverController}
	 */
	public TeleDrive() {
		this.xSupplier = () -> controller.leftX;
		this.ySupplier = () -> controller.leftY;
		this.offenseRotationSupplier = controller.getRotationO();
		this.defenseRotationSupplier = controller.getRotationD();
		this.offense = () -> controller.getControlSet().getAsBoolean();
		this.driveSpeed = () -> controller.getSlowDrive().getAsBoolean() ? 1 : 2;

		addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {
		SmartDashboard.putBoolean("Robot Oriented", false);
	}

	@Override
	public void execute() {
		double x = xSupplier.getAsDouble() * driveSpeed.getAsDouble();
		double y = ySupplier.getAsDouble() * driveSpeed.getAsDouble();
		double rotation;

		if (offense.getAsBoolean()) { // offensive turn ability
			rotation = offenseRotationSupplier.getAsDouble();
			drivetrain.angleAlignDrive(
					x,
					y,
					rotation);
		} else { // defensive turn ability
			rotation = defenseRotationSupplier.getAsDouble();
			drivetrain.drive(
					x,
					y,
					rotation);
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
		SmartDashboard.putBoolean("Robot Oriented", true);
	}
}