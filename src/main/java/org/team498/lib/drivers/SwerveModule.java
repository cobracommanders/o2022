package org.team498.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.lib.util.Falcon500Conversions;

import static org.team498.C2022.Constants.DrivetrainConstants.*;

public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder encoder;
    // offset of the CANCoder in degrees
    private final double angleOffset;
    // TODO test current limiting


    private double lastAngle;

    public SwerveModule(TalonFX driveMotor, TalonFX steerMotor, CANCoder CANCoder, double angleOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.encoder = CANCoder;
        configDriveMotor(driveMotor);
        configSteerMotor(steerMotor);
        configCANCoder(CANCoder);

        this.angleOffset = angleOffset;
        this.lastAngle = getState().angle.getDegrees();

        matchEncoders();
    }


    /** Matches the integrated encoder to the reading from the CANCoder */
    public void matchEncoders() {
        steerMotor.setSelectedSensorPosition(Falcon500Conversions.degreesToFalcon(encoder.getAbsolutePosition() - angleOffset,
                                                                                  MK4I_STEER_REDUCTION_L2));
    }

    /** Return the position of the wheel based on the integrated motor encoder */
    public double getSteerEncoder() {
        return Falcon500Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
                                                    MK4I_STEER_REDUCTION_L2) + angleOffset;
    }

    public double currentSpeedTarget = 0;
    public double currentAngleTarget = 0;
    public boolean forcedAngle = false;

    /** Sets the motors of the swerve module to a provided state */
    public void setState(SwerveModuleState state, boolean force) {
        SwerveModuleState newState = optimize(state, getSteerEncoder());
        this.currentSpeedTarget = newState.speedMetersPerSecond;
        this.currentAngleTarget = newState.angle.getDegrees();
        this.forcedAngle = force;
    }

    @Override
    public void periodic() {
        double velocity = Falcon500Conversions.MPSToFalcon(currentSpeedTarget,
                                                           Units.inchesToMeters(DRIVE_WHEEL_DIAMETER),
                                                           MK4I_DRIVE_REDUCTION_L2);
        driveMotor.set(ControlMode.Velocity, velocity);

        double angle = (Math.abs(velocity) <= MAX_VELOCITY_METERS_PER_SECOND * 0.01) && !forcedAngle
                       ? lastAngle
                       : currentAngleTarget;
        steerMotor.set(ControlMode.Position, Falcon500Conversions.degreesToFalcon(angle - angleOffset, MK4I_STEER_REDUCTION_L2));

        lastAngle = getState().angle.getDegrees();
    }

    /** Get the velocity of the wheel in meters per second */
    private double getVelocityMPS() {
        // Convert the value returned by the sensor (rotations per 100ms) to rotations per second
        return Falcon500Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                                                Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE),
                                                MK4I_DRIVE_REDUCTION_L2);
    }

    /** Get the current state of the swerve module as a {@link SwerveModuleState} */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                // Velocity of the wheel
                getVelocityMPS(),
                // The value of the steering encoder
                Rotation2d.fromDegrees(getSteerEncoder()));
    }

    // Custom optimize method by team 364
    private SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());

        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {targetAngle -= 180;} else {targetAngle += 180;}
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    private void configDriveMotor(TalonFX motor) {
        motor.configFactoryDefault();
        // TalonFXConfiguration currentLimitConfig = new TalonFXConfiguration();

        // currentLimitConfig.supplyCurrLimit.currentLimit = 35;
        // currentLimitConfig.supplyCurrLimit.enable = true;

        // motor.configAllSettings(currentLimitConfig);

        motor.setNeutralMode(NeutralMode.Brake);
        motor.configOpenloopRamp(1);
        motor.setSelectedSensorPosition(0);

        motor.config_kP(0, 0.025);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.5);

        motor.setInverted(true);
    }

    private void configSteerMotor(TalonFX motor) {
        motor.configFactoryDefault();
        // TalonFXConfiguration currentLimitConfig = new TalonFXConfiguration();

        // currentLimitConfig.supplyCurrLimit.currentLimit = 20;
        // currentLimitConfig.supplyCurrLimit.enable = true;

        // motor.configAllSettings(currentLimitConfig);

        motor.setNeutralMode(NeutralMode.Brake);
        motor.configOpenloopRamp(1);
        motor.setSelectedSensorPosition(0);

        motor.setSensorPhase(true);

        motor.config_kP(0, 0.2);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.1);

        motor.setInverted(true);

    }

    private void configCANCoder(CANCoder CANCoder) {
        // Sets the encoder to boot to the absolute position instead of 0
        CANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        // Set the encoder to return values from 0 to 360 instead of -180 to +180
        CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

}