package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import org.team498.lib.util.RotationUtil;

public class Gyro extends ADIS16448_IMU {
    private double angleOffset = 0;

    /** @return yaw angle in degrees (CCW positive), ranging from -180 to 180 degrees */
    @Override
    public synchronized double getAngle() {
        return RotationUtil.toSignedDegrees(super.getAngle() + angleOffset);
    }

    /** @return raw angle unaffected by the offset */
    public double getRawAngle() {
        return RotationUtil.toSignedDegrees(super.getAngle());
    }

    /** @return the rotation offset of the gyro */
    public double getAngleOffset() {
        return angleOffset;
    }

    /**
     * Set an offset to adjust the output of the gyro by.
     *
     * @param angleOffset the offset in degrees
     */
    public void setAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }
}
