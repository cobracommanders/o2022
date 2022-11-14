package org.team498.C2022;

import static org.team498.C2022.Constants.LimelightConstants.kLimelightLensHeight;
import static org.team498.C2022.Constants.LimelightConstants.kLimelightMountAngle;
import static org.team498.C2022.Constants.LimelightConstants.kVisionTapeHeight;
import static org.team498.C2022.Constants.OIConstants.kDriverControllerID;
import static org.team498.C2022.Constants.OIConstants.kOperatorControllerID;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.team498.C2022.commands.CalibrateGyro;
import org.team498.C2022.commands.auto.Auto1;
import org.team498.C2022.commands.drivetrain.CurveFollower;
import org.team498.C2022.commands.drivetrain.FieldOrientedDrive;
import org.team498.C2022.commands.drivetrain.SetPosition;
import org.team498.C2022.commands.drivetrain.SnapDrive;
import org.team498.C2022.commands.drivetrain.WPIDrive;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Vision;
import org.team498.lib.drivers.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
	private final Vision vision = new Vision();
	private final Drivetrain drivetrain = new Drivetrain();

	private final XboxController driverController = new XboxController(kDriverControllerID);
	private final XboxController operatorController = new XboxController(kOperatorControllerID);

	String trajectoryJSON = "output/Unnamed.wpilib.json";
	Trajectory trajectory = new Trajectory();
	public void getFile() {
		Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
		try {
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	public RobotContainer() {
		getFile();
		configureButtonBindings();
		drivetrain.setDefaultCommand(new SnapDrive(this, drivetrain, ()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()-> getControllerAngle(driverController), 0.1, ()-> false));
	}
	public double lastAngle = 0;
	public double offset = 0;
	private double getControllerAngle(XboxController controller) {

		double inputY = (Math.abs(controller.getRightY()) > .1) ? controller.getRightY() : 0.0001;
		double inputX = (Math.abs(controller.getRightX()) > .1) ? -controller.getRightX() : 0.0001;
		if (Math.abs(controller.getRightY()) < .5 && Math.abs(controller.getRightX()) < .5) {
			inputY = 0.0001;
			inputX = 0.0001;
		}
		double angle = Math.toDegrees(Math.atan(inputY / inputX)) + offset;
		if (inputX < 0) {
			angle = angle + Math.copySign(180, inputX);
		}
		if (angle == 45 + offset) {
			angle = lastAngle;
		}
		
		lastAngle = angle;
		SmartDashboard.putNumber("Controller input", angle);
		return angle;
	}
	private double getPOVAngle(XboxController controller) {
		double input = controller.getPOV();
		double result;
		if (input != -1) {
			if (input > 180) {
				result = input - 360;
			}
			else {
				result = input;
			}
		} else {
			result = lastAngle;
		}
		lastAngle = result;
		return result - 90;
	}

	/**
	 * Standard driver controls
	 */
	private void configureButtonBindings() {
		new JoystickButton(driverController, Button.kA.value).whenPressed(new InstantCommand(()-> drivetrain.zeroGyro())).whenPressed(new InstantCommand(()-> drivetrain.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))));
		new JoystickButton(driverController, Button.kB.value).whenActive(new SetPosition(drivetrain, 0, 0, 45));
		new JoystickButton(driverController, Button.kY.value).whenPressed(new InstantCommand()).whenActive(new Auto1(drivetrain, this));
		new JoystickButton(driverController, Button.kX.value).whenActive(new WPIDrive(drivetrain, generateTrajectory()));
		new JoystickButton(driverController, Button.kBack.value).whenActive(new WPIDrive(drivetrain, trajectory));

		
	}
		public Trajectory generateTrajectory() {
			var startPose = new Pose2d(new Translation2d(), new Rotation2d());
			var endPose = new Pose2d(new Translation2d(1, 3), Rotation2d.fromDegrees(90));
			var waypoints = new ArrayList<Translation2d>();
			waypoints.add(new Translation2d(1, 1));
			TrajectoryConfig config = new TrajectoryConfig(2, 3);
			config.setReversed(true);
			return TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);
		}
	public Command getAutoCommand() {
		return null;
	}

	public Command getRobotInitCommand() {
		return new CalibrateGyro(drivetrain);
	}
}