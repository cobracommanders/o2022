package org.team498.C2022.commands.drivetrain.archive;

import java.util.function.DoubleSupplier;

import org.team498.C2022.Constants.DrivetrainConstants;
import org.team498.C2022.DriverController;
import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Controls the robot in field oriented mode
public class OffenseDrive extends CommandBase {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final DriverController controller = DriverController.getInstance();

	private final DoubleSupplier translationXSupplier = controller.getX();
	private final DoubleSupplier translationYSupplier = controller.getY();
	private final DoubleSupplier rotationSupplier = controller.getRotationO();
	private final JoystickButton slowDrive = controller.getSlowDrive();

	public OffenseDrive() {
		addRequirements(this.drivetrain);
	}

	public boolean hasSnappedBefore = false;

	@Override
	public void execute() {
		double driveSpeed = slowDrive.get() ? 0.5 : 1;
		//double driveSpeed = DriverStation.getStickButton(0, Button.kRightBumper.value) ? 0.5 : 1;
		SmartDashboard.putNumber("Slow Drive", driveSpeed);

		double xTranslation = translationXSupplier.getAsDouble() * driveSpeed * DrivetrainConstants.kMaxVelocityMetersPerSecond;
		double yTranslation = translationYSupplier.getAsDouble() * driveSpeed * DrivetrainConstants.kMaxVelocityMetersPerSecond;
		double rotation = rotationSupplier.getAsDouble();

		if (DriverController.getInstance().getRotationActive() || hasSnappedBefore) {
			hasSnappedBefore = true;
			drivetrain.angleAlignDrive(
					xTranslation,
					yTranslation,
					rotation);
		} else {
			drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xTranslation, yTranslation, 0, Rotation2d.fromDegrees(drivetrain.getYaw180())));
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}