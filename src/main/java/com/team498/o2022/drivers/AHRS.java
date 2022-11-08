package com.team498.o2022.drivers;

import com.team254.lib.geometry.Rotation2d;
import com.team498.o2022.Constants;
import com.team498.o2022.Ports;

import edu.wpi.first.wpilibj.I2C.Port;

public class AHRS {

    private static AHRS mInstance;

    public static AHRS getInstance() {
        if (mInstance == null) {
            mInstance = new AHRS(Ports.AHRS);
        }
        return mInstance;
    }

    // Actual AHRS object
    private final com.kauailabs.navx.frc.AHRS mGyro;

    // Configs
    private boolean inverted = Constants.SwerveConstants.invertGyro;
    private Rotation2d yawAdjustmentAngle = Rotation2d.identity();
    private Rotation2d rollAdjustmentAngle = Rotation2d.identity();

    private AHRS(Port port) {        
        mGyro = new com.kauailabs.navx.frc.AHRS(port);
        mGyro.calibrate();
    }

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
        if (inverted) {
            return angle.inverse();
        }
        return angle;
    }

    public Rotation2d getRoll() {
        return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.inverse());
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw());
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll());
    }
}