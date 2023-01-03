package org.team498.C2022;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Intake;
import org.team498.C2022.subsystems.Intake.State;
import org.team498.lib.drivers.ControllerPackage;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriverController extends ControllerPackage {
    private static DriverController mInstance;

    public static DriverController getInstance() {
        if (mInstance == null) {
            mInstance = new DriverController(new XboxController(Constants.OIConstants.kDriverControllerID));
        }
        return mInstance;
    }

    private final XboxController controller;
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier rotationO;
    DoubleSupplier rotationD;
    JoystickButton resetPose;
    JoystickButton controlSet;
    JoystickButton slowDrive;
    Intake.State intakeState;
    double wristState;

    private DriverController(XboxController controller) {
        super(controller);
        this.controller = controller;
        updateDriverSet();
    }

    @Override
    public void periodic() {
        super.periodic();
        updateDriverSet();
    }

    private void updateDriverSet() {
        x = () -> -squareInput(super.leftY);
        y = () -> -squareInput(super.leftX);
        rotationO = () -> updateRotationSetpoint(controller, drivetrain);
        rotationD = () -> squareInput(super.rightX);
        resetPose = super.aButton;
        controlSet = super.leftBumper;
        slowDrive = super.rightBumper;
        intakeState = updateIntakeState();
        wristState = updateWristState();
    }

    public DoubleSupplier getX() {
        return x;
    }

    public DoubleSupplier getY() {
        return y;
    }

    public DoubleSupplier getRotationO() {
        return rotationO;
    }

    public DoubleSupplier getRotationD() {
        return rotationD;
    }

    public JoystickButton getResetPose() {
        return resetPose;
    }

    public JoystickButton getControlSet() {
        return resetPose;
    }

    public BooleanSupplier getSlowDrive() {
        return slowDrive;
    }

    public Intake.State getIntakeState() {
        return intakeState;
    }

    public double getWristState() {
        return wristState;
    }

    private double squareInput(double input) {
        return Math.copySign(Math.pow(input, 2), input);
    }

    private Intake.State updateIntakeState() {
        if (super.leftTrigger) {
            return State.INTAKE;
        } else if (super.rightTrigger) {
            return State.OUTTAKE;
        } else {
            return State.IDLE;
        }
    }

    private double updateWristState() {
        if (super.leftTrigger || super.rightTrigger) {
            return 1;
        } else {
            return 0;
        }
    }

    private double updateRotationSetpoint(XboxController controller, Drivetrain drivetrain) {
        if (isPOVActive()) {
            return getPOVAngle(drivetrain);
        } else {
            return getControllerAngle(drivetrain) + 90;
        }
    }

    private double getControllerAngle(Drivetrain drivetrain) {

        double inputY = (Math.abs(super.rightY) > .1) ? super.rightY : 0.0001;
        double inputX = (Math.abs(super.rightX) > .1) ? -super.rightX : 0.0001;
        if (Math.abs(super.rightY) < .5 && Math.abs(super.rightX) < .5) {
            inputY = 0.0001;
            inputX = 0.0001;
        }
        double angle = Math.toDegrees(Math.atan(inputY / inputX)) + drivetrain.offset;
        if (inputX < 0) {
            angle = angle + Math.copySign(180, inputX);
        }
        if (angle == 45 + drivetrain.offset) {
            angle = drivetrain.lastAngle;
        }

        drivetrain.lastAngle = angle;
        return angle;
    }

    private double getPOVAngle(Drivetrain drivetrain) {
        double input = super.pov;
        double result;
        if (input != -1) {
            if (input > 180) {
                result = input - 360;
            } else {
                result = input;
            }
        } else {
            result = drivetrain.lastAngle;
        }
        drivetrain.lastAngle = result;
        return result - 90;
    }

	public double getMagnitude() {
		return Math.hypot(controller.getRightX(), controller.getRightY());
	}

    private boolean isPOVActive() {
        return super.pov != -1;
    }
}
