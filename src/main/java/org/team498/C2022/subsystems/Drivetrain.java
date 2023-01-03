package org.team498.C2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.SwerveModule;

import static org.team498.C2022.Constants.DrivetrainConstants.*;
import static org.team498.C2022.Constants.PoseConstants;
import static org.team498.C2022.Constants.SnapConstants;
import static org.team498.C2022.Ports.*;

public class Drivetrain extends SubsystemBase {
    // Profiled controller for the rotation of the robot
    private final ProfiledPIDController angleController = new ProfiledPIDController(SnapConstants.P,
                                                                                    SnapConstants.I,
                                                                                    SnapConstants.D,
                                                                                    SnapConstants.CONTROLLER_CONSTRAINTS);
    // Profiled controller for the x position of the robot
    private final ProfiledPIDController xController = new ProfiledPIDController(PoseConstants.P,
                                                                                PoseConstants.I,
                                                                                PoseConstants.D,
                                                                                PoseConstants.CONTROLLER_CONSTRAINTS);
    // Profiled controller for the y position of the robot
    private final ProfiledPIDController yController = new ProfiledPIDController(PoseConstants.P,
                                                                                PoseConstants.I,
                                                                                PoseConstants.D,
                                                                                PoseConstants.CONTROLLER_CONSTRAINTS);
    // Holonomic drive controller to follow trajectories
    private final HolonomicDriveController trajectoryController = new HolonomicDriveController(new PIDController(PoseConstants.P,
                                                                                                                 PoseConstants.I,
                                                                                                                 PoseConstants.D),
                                                                                               new PIDController(PoseConstants.P,
                                                                                                                 PoseConstants.I,
                                                                                                                 PoseConstants.D),
                                                                                               angleController);
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Gyro gyro = new Gyro();

    private Drivetrain() {
        TalonFX FL_Drive = new TalonFX(FL_DRIVE_MOTOR_ID);
        TalonFX FR_Drive = new TalonFX(FR_DRIVE_MOTOR_ID);
        TalonFX BL_Drive = new TalonFX(BL_DRIVE_MOTOR_ID);
        TalonFX BR_Drive = new TalonFX(BR_DRIVE_MOTOR_ID);
        TalonFX FL_Steer = new TalonFX(FL_STEER_MOTOR_ID);
        TalonFX FR_Steer = new TalonFX(FR_STEER_MOTOR_ID);
        TalonFX BL_Steer = new TalonFX(BL_STEER_MOTOR_ID);
        TalonFX BR_Steer = new TalonFX(BR_STEER_MOTOR_ID);
        CANCoder FL_CANCoder = new CANCoder(FL_CANCODER_ID);
        CANCoder FR_CANCoder = new CANCoder(FR_CANCODER_ID);
        CANCoder BL_CANCoder = new CANCoder(BL_CANCODER_ID);
        CANCoder BR_CANCoder = new CANCoder(BR_CANCODER_ID);

        SwerveModule FL_Module = new SwerveModule(FL_Drive, FL_Steer, FL_CANCoder, FL_MODULE_OFFSET);
        SwerveModule FR_Module = new SwerveModule(FR_Drive, FR_Steer, FR_CANCoder, FR_MODULE_OFFSET);
        SwerveModule BL_Module = new SwerveModule(BL_Drive, BL_Steer, BL_CANCoder, BL_MODULE_OFFSET);
        SwerveModule BR_Module = new SwerveModule(BR_Drive, BR_Steer, BR_CANCoder, BR_MODULE_OFFSET);

        // Put all the swerve modules in an array to make using them easier
        swerveModules = new SwerveModule[] {FL_Module, FR_Module, BL_Module, BR_Module};

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(SnapConstants.EPSILON);
        xController.setTolerance(PoseConstants.EPSILON);
        yController.setTolerance(PoseConstants.EPSILON);

        // Set up the kinematics
        double moduleDistance = Units.inchesToMeters(SWERVE_MODULE_DISTANCE_FROM_CENTER);
        Translation2d FL_ModulePosition = new Translation2d(moduleDistance, moduleDistance);
        Translation2d FR_ModulePosition = new Translation2d(moduleDistance, -moduleDistance);
        Translation2d BL_ModulePosition = new Translation2d(-moduleDistance, moduleDistance);
        Translation2d BR_ModulePosition = new Translation2d(-moduleDistance, -moduleDistance);
        kinematics = new SwerveDriveKinematics(FL_ModulePosition, FR_ModulePosition, BL_ModulePosition, BR_ModulePosition);
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getRawAngle()));
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(-getYaw()), getModuleStates());

        if (RobotState.isDisabled()) {matchEncoders();}

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Gyro", getYaw());
        SmartDashboard.putNumber("Snap Setpoint", angleController.getGoal().position);
    }

    public ChassisSpeeds getSpeedsFromTrajectoryState(Trajectory.State goal) {
        return trajectoryController.calculate(getPose(), goal, goal.poseMeters.getRotation());
    }

    public boolean atTrajectoryGoal() {
        return trajectoryController.atReference();
    }


    public Pose2d getPose() {return odometry.getPoseMeters();}

    public void setInitialPose(Pose2d pose) {
        odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getRawAngle()));
        gyro.setAngleOffset(pose.getRotation().getDegrees());
    }

    /** @return true if all three swerve controllers have reached their position goals (x pos, y pos, angle) */
    public boolean atPositionGoals() {
        return xController.atGoal() && yController.atGoal() && angleController.atGoal();
    }

    /**
     * Sets the position goals of the swerve drive.
     *
     * @param x     x position goal in meters
     * @param y     y position goal in meters
     * @param angle angle goal in degrees
     */
    public void setPositionGoals(double x, double y, double angle) {
        xController.setGoal(x);
        yController.setGoal(y);
        angleController.setGoal(angle);
    }

    /** @return the value from the x position controller with a custom input measurement */
    public double calculateXController(double measurement) {return xController.calculate(measurement);}
    /** @return the value from the y position controller with a custom input measurement */
    public double calculateYController(double measurement) {return yController.calculate(measurement);}
    /** @return the value from the snap controller with a custom input measurement */
    public double calculateSnapController(double measurement) {return angleController.calculate(measurement);}

    /**
     * Sets the goal of the snap controller to a specified target.
     *
     * @param goal the goal to set in degrees
     */
    public void setSnapGoal(double goal) {angleController.setGoal(goal);}
    /** Calculate the rotational speed from the pid controller, unless it's already at the goal */
    public double calculateSnapSpeed() {
        return angleController.atGoal() ? 0 : angleController.calculate(Math.toRadians(getYaw()));
    }
    /** @return true if the snap controller is at it's goal */
    public boolean atSnapGoal() {return angleController.atGoal();}

    /** Sets the swerve drive to move towards the values specified by the position controllers. */
    public void driveToPositionGoals() {
        // TODO Figure out why x/y between PID controller and odometry pose are reversed
        double xAdjustment = xController.calculate(getPose().getY());
        double yAdjustment = -yController.calculate(getPose().getX());
        double angleAdjustment = angleController.calculate(getYaw());
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(xAdjustment, yAdjustment, angleAdjustment, Rotation2d.fromDegrees(getYaw())));
    }

    /**
     * Sets the swerve drive to desired speed of direction and rotation.
     *
     * @param chassisSpeeds drive speeds to set
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, new Translation2d());
    }

    /**
     * Sets the swerve drive to desired speed of direction and rotation, with the option to use a custom center of rotation.
     *
     * @param chassisSpeeds drive speeds to set
     * @param rotation      a {@link Translation2d} representing the distance from the center of the robot to the desired center
     *                      of rotation
     */
    public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotation) {
        // Use the kinematics to set the desired speed and angle for each swerve module using the input velocities for direction and rotation
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds, rotation), false);
    }

    public void matchEncoders() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.matchEncoders();
        }
    }

    /**
     * Set the swerve modules to the desired states. Under normal operation, the angle will not change unless the robot is moving,
     * however this can be overridden by setting 'force' to true.
     *
     * @param moduleStates an array of {@link SwerveModuleState module states} to set the swerve to
     * @param force        if set to true, the module states will be set even if the robot is not moving
     */
    public void setModuleStates(SwerveModuleState[] moduleStates, boolean force) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

        // Set the motors of the swerve module to the calculated state
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(moduleStates[i], force);
        }
    }

    /** @return an array of {@link SwerveModuleState module states} representing each of the modules */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }


    public void zeroGyro() {gyro.reset();}

    public void calibrateGyro() {gyro.calibrate();}

    /** @return The current yaw angle in degrees (-180 to 180) */
    public double getYaw() {
        return gyro.getAngle();
    }

    public void resetSnapController() {
        angleController.reset(Math.toRadians(getYaw()));
    }


    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

}
