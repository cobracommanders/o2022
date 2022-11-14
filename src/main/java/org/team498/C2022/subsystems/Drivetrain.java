package org.team498.C2022.subsystems;

import static org.team498.C2022.Constants.DrivetrainConstants.kBackLeftDriveMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackLeftEncoderID;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackLeftModuleOffset;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackLeftSteerMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackRightDriveMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackRightEncoderID;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackRightModuleOffset;
import static org.team498.C2022.Constants.DrivetrainConstants.kBackRightSteerMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontLeftDriveMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontLeftEncoderID;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontLeftModuleOffset;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontLeftSteerMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontRightDriveMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontRightEncoderID;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontRightModuleOffset;
import static org.team498.C2022.Constants.DrivetrainConstants.kFrontRightSteerMotorID;
import static org.team498.C2022.Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond;
import static org.team498.C2022.Constants.DrivetrainConstants.kSwerveModuleDistanceFromCenter;

import com.kauailabs.navx.frc.AHRS;

import org.team498.C2022.Constants;
import org.team498.C2022.RobotContainer;
import org.team498.lib.drivers.SwerveModule;
import org.team498.lib.util.TimeDelayedBoolean;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

	public ProfiledPIDController snapController = new ProfiledPIDController(
		Constants.SnapConstants.kP,
		Constants.SnapConstants.kI, 
		Constants.SnapConstants.kD,
		Constants.SnapConstants.kThetaControllerConstraints
	);
	public ProfiledPIDController xController = new ProfiledPIDController(		
		Constants.PoseConstants.kP,
		Constants.PoseConstants.kI, 
		Constants.PoseConstants.kD,
		Constants.PoseConstants.kThetaControllerConstraints);
	public ProfiledPIDController yController = new ProfiledPIDController(		
		Constants.PoseConstants.kP,
		Constants.PoseConstants.kI, 
		Constants.PoseConstants.kD,
		Constants.PoseConstants.kThetaControllerConstraints);
	public boolean isSnapping = false;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;
	private final SwerveModule[] swerveModules;

	private static SwerveDriveKinematics kinematics;

	// Distance of the swerve modules from the center of the robot converted to
	// meters
	private final double moduleDistance = Units.inchesToMeters(kSwerveModuleDistanceFromCenter);

	// Create positions for the swerve modules relative to the center of the robot

	// This is used by the kinematics and would be mainly helpful for determining
	// rotation on a non-square base or with more than 4 swerve modules
	private final Translation2d frontLeftModulePosition = new Translation2d(moduleDistance, moduleDistance);
	private final Translation2d frontRightModulePosition = new Translation2d(moduleDistance, -moduleDistance);
	private final Translation2d backLeftModulePosition = new Translation2d(-moduleDistance, moduleDistance);
	private final Translation2d backRightModulePosition = new Translation2d(-moduleDistance, -moduleDistance);

	public final SwerveDriveOdometry odometry;

	//public static AHRS IMU = new AHRS(Port.kUSB1);
	public final ADIS16448_IMU IMU = new ADIS16448_IMU();

	private final Field2d field = new Field2d();
	private double offset = 0;

	public void setGyroOffset(double offset) {
		IMU.reset();
		this.offset = offset;
	}

	public synchronized void zeroGyro() {
		IMU.reset();
	}

	public synchronized void calibrateGyro() {
		IMU.calibrate();
	}

	public Drivetrain() {
		snapController.enableContinuousInput(-Math.PI, Math.PI);
		frontLeft = new SwerveModule(
				// Drive motor ID
				kFrontLeftDriveMotorID,
				// Steer Motor ID
				kFrontLeftSteerMotorID,
				// CANCoder ID
				kFrontLeftEncoderID,
				// Offset
				kFrontLeftModuleOffset);

		// Same for the rest of them
		frontRight = new SwerveModule(kFrontRightDriveMotorID, kFrontRightSteerMotorID, kFrontRightEncoderID,
				kFrontRightModuleOffset);
		backLeft = new SwerveModule(kBackLeftDriveMotorID, kBackLeftSteerMotorID, kBackLeftEncoderID,
				kBackLeftModuleOffset);
		backRight = new SwerveModule(kBackRightDriveMotorID, kBackRightSteerMotorID, kBackRightEncoderID,
				kBackRightModuleOffset);

		// Create an array of all the swerve modules to make editing them easier
		swerveModules = new SwerveModule[] {
				frontLeft,
				frontRight,
				backLeft,
				backRight
		};

		// Setup the kinematics
		kinematics = new SwerveDriveKinematics(
				frontLeftModulePosition,
				frontRightModulePosition,
				backLeftModulePosition,
				backRightModulePosition);

		odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));

		matchEncoders();

		resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
	}
	public double calculateSnapValue() {
        return snapController.calculate(Math.toRadians(getYaw180()));
    }

    public void startSnap(double snapAngle) {
        snapController.reset(Math.toRadians(getYaw180()));
        snapController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }
    
    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    public boolean snapComplete() {
        double error = snapController.getGoal().position - Math.toRadians(getYaw180());
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon), Constants.SnapConstants.kTimeout);
    }

    public void maybeStopSnap(boolean force){
        if (!isSnapping) {
            return;
        } 
        if (force || snapComplete()) {
            isSnapping = false;
            snapController.reset(Math.toRadians(getYaw180()));
        }
    }
	public synchronized void matchEncoders() {
		for (SwerveModule swerveModule : swerveModules) {
			swerveModule.matchEncoders();
		}
	}

	public static SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	public synchronized Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public synchronized void resetOdometry(Pose2d pose) {
		odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
	}

	public synchronized void setModuleStates(SwerveModuleState[] moduleStates) {
		setModuleStates(moduleStates, false);
	}

	public synchronized void setModuleStates(SwerveModuleState[] moduleStates, boolean force) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				// The optimized states
				moduleStates,
				// The maximum velocity of the robot
				kMaxVelocityMetersPerSecond);

		for (int i = 0; i < swerveModules.length; i++) {
			// Set the motors of the swerve module to the calculated state
			swerveModules[i].setState(moduleStates[i], force);
		}
	}
	public void autoDrive(Trajectory trajectory) {
		
	}
	public HolonomicDriveController driveController = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), snapController);
	// public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative) {
    //     snapController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
    //     double angleAdjustment = snapController.calculate(Math.toRadians(getYaw180()));
	// 	drive(ChassisSpeeds.fromFieldRelativeSpeeds(translation2d.getX(), translation2d.getY(), angleAdjustment, Rotation2d.fromDegrees(getYaw180())));
    //     //drive(translation2d, angleAdjustment, fieldRelative, false);
    // }
	public void angleAlignDrive(double x, double y, double targetHeading, boolean fieldRelative, RobotContainer container) {
        snapController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
        double angleAdjustment = snapController.calculate(Math.toRadians(getYaw180()));
		if (snapController.atGoal()) {
			angleAdjustment = 0;
		}
		SmartDashboard.putNumber("Angle adjustment", angleAdjustment);
		drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, angleAdjustment, Rotation2d.fromDegrees(getYaw180() + container.offset)));
		//drive(translation2d, angleAdjustment, fieldRelative, false);
    }
	public boolean atPosition() {
		return xController.atGoal() && yController.atGoal() && snapController.atGoal();
	}
	public void setPosition(double x, double y, double angle) {
		xController.setGoal(new TrapezoidProfile.State(x, 0));
		yController.setGoal(new TrapezoidProfile.State(y, 0));
		snapController.setGoal(new TrapezoidProfile.State(Math.toRadians(angle), 0.0));

		double xAdjustment = xController.calculate(getPose().getY());
		double yAdjustment = -yController.calculate(getPose().getX());
        double angleAdjustment = snapController.calculate(Math.toRadians(getYaw180()));
		drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAdjustment, yAdjustment, angleAdjustment, Rotation2d.fromDegrees(getYaw180())));

	}
	public synchronized void drive(ChassisSpeeds chassisSpeeds) {
		drive(chassisSpeeds, new Translation2d(0, 0));
	}

	// Method to set the swerve drive to desired speed of direction and rotation
	public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotation) {
		// Use the kinematics to set the desired speed and angle for each swerve module
		// using the input velocities for direction and rotation
		setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds, rotation), false);
	}

	/**
	 * @return The current yaw angle in degrees (-180 to 180)
	 */
	public synchronized double getYaw180() {
		return limit180(-IMU.getAngle() - 90);
	}
	public double limit180(double value) {
		value %= 360;
		if (value > 180) {
			value -= 360;
		} else if (value <= -180) {
			value += 360;
		}
		return value;
	}

	/**
	 * @return The total accumulated yaw angle in degrees
	 */
	public synchronized double getYaw() {
		return IMU.getAngle();
	}

	// Return an array of all the module states
	public synchronized SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				frontLeft.getState(),
				frontRight.getState(),
				backLeft.getState(),
				backRight.getState()
		};
	}

	public synchronized double getSpeedX() {
		ChassisSpeeds speed = kinematics.toChassisSpeeds(getModuleStates());
		return speed.vxMetersPerSecond;
	}
	public synchronized double getSpeedY() {
		ChassisSpeeds speed = kinematics.toChassisSpeeds(getModuleStates());
		return speed.vyMetersPerSecond;
	}

	public synchronized boolean isMoving() {
		ChassisSpeeds speed = kinematics.toChassisSpeeds(getModuleStates());
		return ((Math.abs(speed.vxMetersPerSecond) + Math.abs(speed.vyMetersPerSecond)
				+ Math.abs(speed.omegaRadiansPerSecond)) > 0.1);
	}

	// Set a number to 0 which counts up to 500 every 10 seconds to reset the
	// encoders
	private int encoderResetTimer = 0;
	private int idleResetTimer = 0;

	@Override
	public void periodic() {
		odometry.update(Rotation2d.fromDegrees(-getYaw() + offset), getModuleStates());
		field.setRobotPose(getPose());
		if (encoderResetTimer++ > 500 && !isMoving() && idleResetTimer++ > 50) {
			matchEncoders();
			encoderResetTimer = 0;
			idleResetTimer = 0;
		}
		if (isMoving()) {
			idleResetTimer = 0;
		}
		SmartDashboard.putNumber("Angle Setpoint", snapController.getGoal().position);
		SmartDashboard.putBoolean("moving", isMoving());
		SmartDashboard.putData(field);
		SmartDashboard.putNumber("gyro", getYaw180());
		SmartDashboard.putData(this);
	}
}
