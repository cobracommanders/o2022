package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class Gyro extends ADIS16448_IMU {
    private double angleOffset = 0;

    @Override
    public synchronized double getAngle() {
        return super.getAngle() + angleOffset;
    }

    /** @return raw angle unaffected by the offset */
    public double getRawAngle() {
        return super.getAngle();
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public void setAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }
}
