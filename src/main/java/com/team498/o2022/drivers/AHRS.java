package com.team498.o2022.drivers;

import com.team254.lib.geometry.Rotation2d;
import com.team498.o2022.Constants;
import com.team498.o2022.Ports;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.I2C.Port;

public class AHRS {

    private static AHRS mInstance;

    public static AHRS getInstance() {
        if (mInstance == null) {
            mInstance = new AHRS();
        }
        return mInstance;
    }

    // Actual AHRS object
    private final ADIS16448_IMU mGyro;

    // Configs
    private boolean inverted = Constants.SwerveConstants.invertGyro;
    private Rotation2d yawAdjustmentAngle = Rotation2d.identity();
    private Rotation2d rollAdjustmentAngle = Rotation2d.identity();

    private AHRS() {        
        mGyro = new ADIS16448_IMU();
        mGyro.calibrate();
    }

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
        if (inverted) {
            return angle.inverse();
        }
        return angle;
    }






}