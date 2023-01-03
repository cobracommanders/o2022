package org.team498.C2022;

import org.team498.C2022.commands.CalibrateGyro;
import org.team498.C2022.commands.auto.Auto_1;
import org.team498.C2022.commands.drivetrain.TeleDrive;
import org.team498.C2022.commands.drivetrain.archive.SnapDrive;
import org.team498.C2022.commands.wrist.SetWrist;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
	private static RobotContainer mInstance;

	public static RobotContainer getInstance() {
		if (mInstance == null) {
			mInstance = new RobotContainer();
		}
		return mInstance;
	}

	// private final Vision vision = new Vision();
	private final Drivetrain drivetrain = Drivetrain.getInstance();

	private final DriverController driverControls = DriverController.getInstance();

	public RobotContainer() {
		configureDriverBindings();
		drivetrain.setDefaultCommand(new SnapDrive(drivetrain, driverControls.getX(), driverControls.getY(), driverControls.getRotationO(), 0.1, driverControls.getSlowDrive()));
	}

	private void configureDriverBindings() {
		//driverControls.aButton.whenPressed(new SetWrist(Wrist.State.OUT));
		//driverControls.bButton.whenPressed(new SetWrist(Wrist.State.IN));
		driverControls.aButton.whenPressed(new InstantCommand(() -> drivetrain.IMU.reset()));
	}

	public Command getAutoCommand() {
		return new Auto_1(drivetrain);
	}

	public Command getRobotInitCommand() {
		return new CalibrateGyro(drivetrain);
	}
}
