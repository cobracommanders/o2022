package org.team498.C2022.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team498.C2022.DriverController;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Shooter;

// Controls the robot in fiels oriented mode
public class TeleRobot extends CommandBase {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Shooter shooter = Shooter.getInstance();
	private final DriverController controller = DriverController.getInstance();

	private final DoubleSupplier xSupplier;
	private final DoubleSupplier ySupplier;
	private final DoubleSupplier offenseRotationSupplier;
	private final DoubleSupplier defenseRotationSupplier;
	private final BooleanSupplier offense;
	private final DoubleSupplier driveSpeed;
    private final BooleanSupplier isShooting;

	/**
	 * requires {@link Drivetrain drivetrain} and uses {@link DriverController}
	 */
	public TeleRobot() {
		this.xSupplier = () -> controller.leftX;
		this.ySupplier = () -> controller.leftY;
		this.offenseRotationSupplier = controller.getRotationO();
		this.defenseRotationSupplier = controller.getRotationD();
		this.offense = () -> controller.getControlSet().getAsBoolean();
		this.driveSpeed = () -> controller.getSlowDrive().getAsBoolean() ? 1 : 2;
        this.isShooting = controller.getIsShooting();

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
        //use a while loop so that x and y speeds lock for consistent results
        while (isShooting.getAsBoolean()) {
            Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
            drivetrain.targetPointDrive(x, y, target);
            shooter.setVisionSpeed(Math.hypot(x - target.getX(), y - target.getY()));
        }

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