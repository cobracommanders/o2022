package org.team498.C2022;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final class OIConstants {
		public static final int kDriverControllerID = 0;
		public static final int kOperatorControllerID = 1;
		public static final double kControllerRumbleRange = 150;
	}

	public static final class WristConstants {
		public static final double kInPosition = 0;
		public static final double kOutPosition = 2;
		public static final int leftID = 31;
		public static final int rightID = 37;
		public static final double kP = 0.6;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kMaxVelocity = 3;
		public static final double kMaxAcceleration = Math.pow(kMaxVelocity, 2);
	}

	public static final class IntakeContants {
		public static final double kIntakeSpeed = 1;
		public static final double kOuttakeSpeed = -1;
		public static final double kIdleSpeed = 0;
	}

	public static final class LimelightConstants {
		public static final double kVisionTapeHeight = 101.625;
		public static final double kLimelightMountAngle = 30;
		public static final double kLimelightLensHeight = 0.247776 + 1.875 + 24; // May be slightly over
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPThetaController = 1;
	}

	public static final class DrivetrainConstants {
		// This is just copied from SDS, we need to find the real one still

		public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 *
				((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)) * Units.inchesToMeters(4) *
				Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;

		// public static final double kMaxVelocityMetersPerSecond = 2;

		public static final double kSwerveModuleDistanceFromCenter = 10.75;

		public static final double kMK4IDriveReductionL2 = 6.75;
		public static final double kMK4ISteerReductionL2 = 21.428571428571428571428571428571; // 150 / 7

		public static final double kDriveWheelDiameter = 4;
		public static final double kDriveWheelCircumference = kDriveWheelDiameter * Math.PI;

		public static final int kFrontLeftDriveMotorID = 3;
		public static final int kFrontRightDriveMotorID = 5;
		public static final int kBackLeftDriveMotorID = 1;
		public static final int kBackRightDriveMotorID = 7;

		public static final int kFrontLeftSteerMotorID = 4;
		public static final int kFrontRightSteerMotorID = 6;
		public static final int kBackLeftSteerMotorID = 2;
		public static final int kBackRightSteerMotorID = 8;

		public static final int kFrontRightEncoderID = 13;
		public static final int kFrontLeftEncoderID = 12;
		public static final int kBackRightEncoderID = 14;
		public static final int kBackLeftEncoderID = 11;

		public static final double kFrontLeftModuleOffset = 137.1;
		public static final double kFrontRightModuleOffset = 312.4;
		public static final double kBackLeftModuleOffset = 74.5;
		public static final double kBackRightModuleOffset = 28.0;
	}

	public static final class SnapConstants {
		public static final double kP = 5.0;
		public static final double kI = 0;
		public static final double kD = 0.0;
		public static final double kTimeout = 0.25;
		public static final double kEpsilon = 1.0;

		// Constraints for the profiled angle controller
		public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond,
				2);

		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class PoseConstants {
		public static final double kP = 2.5;
		public static final double kI = 0;
		public static final double kD = 0.0;
		public static final double kTimeout = 0.25;
		public static final double kEpsilon = .1;

		// Constraints for the profiled angle controller
		public static final double kMaxSpeedPerSecond = 2;
		public static final double kMaxSpeedPerSecondSquared = Math.pow(kMaxSpeedPerSecond, 2);

		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxSpeedPerSecond, kMaxSpeedPerSecondSquared);
	}

	public static final String kRoborioTrajectoryFilepath = "/home/lvuser/paths";
}
