package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team498.lib.util.RotationUtil;

import java.util.EnumMap;

public class Xbox {
    private final int port;
    private double deadzone = 0;
    private double triggerThreshold = 0.1;
    private final EnumMap<Button, Trigger> buttonTriggers = new EnumMap<>(Button.class);
    // TODO use computeIfAbsent for POV triggers and active triggers


    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public Xbox(int port) {
        this.port = port;
    }

    public void setDeadzone(double deadzone) {this.deadzone = deadzone;}
    public void setTriggerThreshold(double triggerThreshold) {this.triggerThreshold = triggerThreshold;}

    public enum Button {
        LeftBumper(5),
        RightBumper(6),
        LeftStick(9),
        RightStick(10),
        A(1),
        B(2),
        X(3),
        Y(4),
        Back(7),
        Start(8),
        LeftTrigger(9),
        RightTrigger(10);


        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        LeftX(0),
        RightX(4),
        LeftY(1),
        RightY(5),
        LeftTrigger(2),
        RightTrigger(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public Trigger POVActive() {return new Trigger(() -> getRawPOV() == -1);}
    public Trigger RightStickActive() {return new Trigger(() -> Math.hypot(getAxis(Axis.RightY), getAxis(Axis.RightX)) != 0);}
    public Trigger LeftStickActive() {return new Trigger(() -> Math.hypot(getAxis(Axis.LeftY), getAxis(Axis.LeftX)) != 0);}


    /* * * * * * * * * * * * * * * * * * * * * * * */
    /* * * * * * * * * * BUTTONS * * * * * * * * * */
    /* * * * * * * * * * * * * * * * * * * * * * * */

    /**
     * Get the current value of a button.
     *
     * @param button the button to read
     * @return the value of the button
     */
    public boolean getButton(Button button) {return DriverStation.getStickButton(port, (byte) button.value);}
    public Trigger getButtonTrigger(Button button) {return new Trigger(() -> getButton(button));}


    /** @return a trigger object using the controller's left bumper */
    public Trigger leftBumper() {return buttonTriggers.computeIfAbsent(Button.LeftBumper, this::getButtonTrigger);}

    /** @return a trigger object using the controller's right bumper */
    public Trigger rightBumper() {return buttonTriggers.computeIfAbsent(Button.RightBumper, this::getButtonTrigger);}

    /** @return a trigger object using the controller's left joystick button */
    public Trigger leftStick() {return buttonTriggers.computeIfAbsent(Button.LeftStick, this::getButtonTrigger);}

    /** @return a trigger object using the controller's right joystick button */
    public Trigger rightStick() {return buttonTriggers.computeIfAbsent(Button.RightStick, this::getButtonTrigger);}

    /** @return a trigger object using the controller's A button */
    public Trigger A() {return buttonTriggers.computeIfAbsent(Button.A, this::getButtonTrigger);}

    /** @return a trigger object using the controller's B button */
    public Trigger B() {return buttonTriggers.computeIfAbsent(Button.B, this::getButtonTrigger);}

    /** @return a trigger object using the controller's X button */
    public Trigger X() {return buttonTriggers.computeIfAbsent(Button.X, this::getButtonTrigger);}

    /** @return a trigger object using the controller's Y button */
    public Trigger Y() {return buttonTriggers.computeIfAbsent(Button.Y, this::getButtonTrigger);}

    /** @return a trigger object using the controller's back button */
    public Trigger back() {return buttonTriggers.computeIfAbsent(Button.Back, this::getButtonTrigger);}

    /** @return a trigger object using the controller's start button */
    public Trigger start() {return buttonTriggers.computeIfAbsent(Button.Start, this::getButtonTrigger);}

    /** @return a trigger object using the controller's right trigger */
    public Trigger rightTrigger() {
        return buttonTriggers.computeIfAbsent(Button.RightTrigger,
                                              k -> new Trigger(() -> getRawAxis(Axis.RightTrigger) > triggerThreshold));
    }

    /** @return a trigger object using the controller's left trigger */
    public Trigger leftTrigger() {
        return buttonTriggers.computeIfAbsent(Button.LeftTrigger,
                                              k -> new Trigger(() -> getRawAxis(Axis.LeftTrigger) > triggerThreshold));
    }


    /* * * * * * * * * * * * * * * * * * * * * * */
    /* * * * * * * * * * AXIS  * * * * * * * * * */
    /* * * * * * * * * * * * * * * * * * * * * * */

    /**
     * Get the current value of an axis.
     *
     * @param axis the axis to read
     * @return the value of the axis
     */
    public double getAxis(Axis axis) {
        double rawAxis = DriverStation.getStickAxis(port, axis.value);
        return Math.abs(rawAxis) > deadzone ? rawAxis : 0;
    }

    /** @return the controller's left X axis */
    public double leftX() {return getAxis(Axis.LeftX);}

    /** @return the controller's left Y axis */
    public double leftY() {return getAxis(Axis.LeftY);}

    /** @return the controller's right X axis */
    public double rightX() {return getAxis(Axis.RightX);}

    /** @return the controller's right Y axis */
    public double rightY() {return getAxis(Axis.RightY);}


    //TODO make left 90 instead 0f 270
    private double lastAngleRight = 0;
    /**
     * Gets the angle of the right joystick, from -180 to 180. Upwards is 0 degrees, right is 90, etc. This value uses the last
     * read angle when the joystick is returned to the center.
     *
     * @return the angle of the right joystick
     */
    public double rightAngle() {
        double x = getAxis(Axis.RightX);
        double y = getAxis(Axis.RightY);
        if (Math.abs(x) < 0.5 && Math.abs(y) < 0.5) return lastAngleRight;

        double result = Math.toDegrees(Math.atan2(-x, -y));

        if (result < 0) {
            result += 360;
        }

        result = RotationUtil.toSignedDegrees(result);

        lastAngleRight = result;
        return result;
    }


    private double lastAngleLeft = 0;
    /**
     * Gets the angle of the left joystick, from -180 to 180. Upwards is 0 degrees, right is 90, etc. This value uses the last
     * read angle when the joystick is returned to the center.
     *
     * @return the angle of the left joystick
     */
    public double leftAngle() {
        double x = getAxis(Axis.LeftX);
        double y = getAxis(Axis.LeftY);
        if (Math.abs(x) < 0.5 && Math.abs(y) < 0.5) return lastAngleLeft;

        double result = Math.toDegrees(Math.atan2(-x, -y));

        if (result < 0) {
            result += 360;
        }

        result = RotationUtil.toSignedDegrees(result);

        lastAngleLeft = result;
        return result;
    }

    /**
     * Get the current raw value of an axis, unmodified by the deadzone.
     *
     * @param axis the axis to read
     * @return the value of the axis
     */
    public double getRawAxis(Axis axis) {
        return DriverStation.getStickAxis(port, axis.value);
    }

    /* * * * * * * * * * * * * * * * * * * * * */
    /* * * * * * * * * * POV * * * * * * * * * */
    /* * * * * * * * * * * * * * * * * * * * * */

    //TODO triggers for each angle of the POV

    /**
     * Get the current value from the POV.
     *
     * @return the value of the POV
     */
    public int getRawPOV() {
        return DriverStation.getStickPOV(port, 0);
    }

    private double lastAnglePOV = -0.0;
    /**
     * Gets the angle of the POV, from -180 to 180. Upwards is 0 degrees, right is 90, etc. This value uses the last read angle
     * when the POV is returned to the center.
     *
     * @return the angle of the POV
     */
    public double POVAngle() {
        double angle = getRawPOV();
        if (angle == -1) return lastAnglePOV;

        angle = -RotationUtil.toSignedDegrees(angle);

        lastAnglePOV = angle;
        return angle;
    }

    public Trigger POV0() {return new Trigger(() -> POVAngle() == -0);}
    public Trigger POV45() {return new Trigger(() -> POVAngle() == 45);}
    public Trigger POV90() {return new Trigger(() -> POVAngle() == 90);}
    public Trigger POV135() {return new Trigger(() -> POVAngle() == 135);}
    public Trigger POV180() {return new Trigger(() -> POVAngle() == -180);}
    public Trigger POVMinus135() {return new Trigger(() -> POVAngle() == -135);}
    public Trigger POVMinus90() {return new Trigger(() -> POVAngle() == -90);}
    public Trigger POVMinus45() {return new Trigger(() -> POVAngle() == -45);}
}

