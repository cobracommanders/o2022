package org.team498.C2022;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_ID = 0;
    }

    public static final class DrivetrainConstants {

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5;

        public static final double SWERVE_MODULE_DISTANCE_FROM_CENTER = 10.75;

        public static final double MK4I_DRIVE_REDUCTION_L2 = 6.75;
        public static final double MK4I_STEER_REDUCTION_L2 = 21.428571428571428571428571428571; // 150 / 7

        public static final double DRIVE_WHEEL_DIAMETER = 4;
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * Math.PI;

        public static final double FL_MODULE_OFFSET = 137.1;
        public static final double FR_MODULE_OFFSET = 312.4;
        public static final double BL_MODULE_OFFSET = 74.5;
        public static final double BR_MODULE_OFFSET = 28.0;
    }

    public static final class SnapConstants {
        public static final double P = 5;
        public static final double I = 0;
        public static final double D = 0;
        public static final double EPSILON = 1.0;


        // Constraints for the profiled angle controller
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.pow(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2);

        public static final TrapezoidProfile.Constraints CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    }

    public static final class PoseConstants {
        public static final double P = 2.5;
        public static final double I = 0;
        public static final double D = 0;
        public static final double EPSILON = 0;

        // Constraints for the profiled angle controller
        public static final double MAX_SPEED_PER_SECOND = 2;
        public static final double MAX_SPEED_PER_SECOND_SQUARED = Math.pow(MAX_SPEED_PER_SECOND, 2);

        public static final TrapezoidProfile.Constraints CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_SPEED_PER_SECOND, MAX_SPEED_PER_SECOND_SQUARED);
    }
}
