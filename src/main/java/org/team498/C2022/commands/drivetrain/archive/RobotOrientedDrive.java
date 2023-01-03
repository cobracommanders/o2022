package org.team498.C2022.commands.drivetrain.archive;

import java.util.function.DoubleSupplier;

import org.team498.C2022.Constants.DrivetrainConstants;
import org.team498.C2022.DriverController;
import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Controls the robot in field oriented mode
public class RobotOrientedDrive extends CommandBase {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final DriverController controller = DriverController.getInstance();

	private final DoubleSupplier translationXSupplier = controller.getX();
	private final DoubleSupplier translationYSupplier = controller.getY();
	private final DoubleSupplier rotationSupplier = controller.getRotationD();
	private final JoystickButton slowDrive = controller.getSlowDrive();

	public RobotOrientedDrive() {
		addRequirements(this.drivetrain);
	}

	@Override
	public void execute() {
		double driveSpeed = slowDrive.get() ? 0.5 : 1;

		double xTranslation = translationXSupplier.getAsDouble() * driveSpeed * DrivetrainConstants.kMaxVelocityMetersPerSecond;
		double yTranslation = translationYSupplier.getAsDouble() * driveSpeed * DrivetrainConstants.kMaxVelocityMetersPerSecond;
		double rotation = rotationSupplier.getAsDouble() * DrivetrainConstants.kMaxAngularSpeedRadiansPerSecond;

		drivetrain.drive(new ChassisSpeeds(xTranslation, yTranslation, rotation));
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}