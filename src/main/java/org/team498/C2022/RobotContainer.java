package org.team498.C2022;

import static org.team498.C2022.Constants.OIConstants.kDriverControllerID;
import static org.team498.C2022.Constants.OIConstants.kOperatorControllerID;

import java.util.Set;

import org.team498.C2022.commands.CalibrateGyro;
import org.team498.C2022.commands.auto.Auto_1;
import org.team498.C2022.commands.drivetrain.FieldOrientedDrive;
import org.team498.C2022.commands.drivetrain.SnapDrive;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Intake;
import org.team498.C2022.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RobotContainer {
	private final Vision vision = new Vision();
	private final Drivetrain drivetrain = new Drivetrain();
	private final Intake intake = new Intake();

	private final XboxController driverController = new XboxController(kDriverControllerID);
	private final XboxController operatorController = new XboxController(kOperatorControllerID);

	private final DriverController driverControls = new DriverController(driverController, drivetrain);

	public RobotContainer() {
		configureDriverBindings();
		drivetrain.setDefaultCommand(new SnapDrive(drivetrain, driverControls.getX(), driverControls.getY(), driverControls.getRotationO(), 0.1, driverControls.getSlowDrive()));
		intake.setDefaultCommand(new InstantCommand(()-> intake.setState(driverControls.getIntakeState()), intake));
	}
	
	

	/**
	 * Standard driver controls
	 */
	private void configureDriverBindings() {
		driverControls.getControlSet().toggleWhenActive(new InstantCommand(()-> drivetrain.setDefaultCommand(new FieldOrientedDrive(drivetrain, driverControls.getY(), driverControls.getX(), driverControls.getRotationD(), 0.1, driverControls.getSlowDrive()))));
		driverControls.getControlSet().negate().toggleWhenActive(new InstantCommand(()-> drivetrain.setDefaultCommand(new SnapDrive(drivetrain, driverControls.getX(), driverControls.getY(), driverControls.getRotationO(), 0.1, driverControls.getSlowDrive()))));
		driverControls.getResetPose().whenPressed(new InstantCommand(()-> drivetrain.zeroGyro())).whenPressed(new InstantCommand(()-> drivetrain.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
	}

	public Command getAutoCommand() {
		return new Auto_1(drivetrain);
	}

	public Command getRobotInitCommand() {
		return new CalibrateGyro(drivetrain);
	}
}