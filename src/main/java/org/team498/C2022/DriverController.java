package org.team498.C2022;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team498.C2022.subsystems.Drivetrain;
import org.team498.lib.drivers.ControllerPackage;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriverController extends ControllerPackage {
    private final XboxController controller;
    private final Drivetrain drivetrain;
    public DriverSet set;

    class DriverSet {
        DoubleSupplier x;
        DoubleSupplier y;
        DoubleSupplier rotationO;
        DoubleSupplier rotationD;
        JoystickButton resetPose;
        JoystickButton controlSet;
        JoystickButton slowDrive;
    }

    public DriverController(XboxController controller, Drivetrain drivetrain) {
        super(controller);
        this.controller = controller;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        updateControllerSet();
        updateDriverSet();
    }
    private void updateDriverSet() {
        set.x = ()-> squareInput(-super.set.leftY);
        set.y = ()-> squareInput(-super.set.leftX);
        set.rotationO = ()-> getRotationSetpoint(controller, drivetrain);
        set.rotationD = ()-> squareInput(super.set.rightX);
        set.resetPose = super.set.aButton;
        set.controlSet = super.set.leftBumper;
        set.slowDrive = super.set.rightBumper;
    }
    public DoubleSupplier getX() {
        return set.x;
    }
    public DoubleSupplier getY() {
        return set.y;
    }
    public DoubleSupplier getRotationO() {
        return set.rotationO;
    }
    public DoubleSupplier getRotationD() {
        return set.rotationD;
    }
    public JoystickButton getResetPose() {
        return set.resetPose;
    }
    public JoystickButton getControlSet() {
        return set.resetPose;
    }
    public BooleanSupplier getSlowDrive() {
        return set.slowDrive;
    }
    private double squareInput(double input) {
        return Math.copySign(Math.pow(input, 2), input);
    }
	public static double getControllerAngle(XboxController controller, Drivetrain drivetrain) {

		double inputY = (Math.abs(controller.getRightY()) > .1) ? controller.getRightY() : 0.0001;
		double inputX = (Math.abs(controller.getRightX()) > .1) ? -controller.getRightX() : 0.0001;
		if (Math.abs(controller.getRightY()) < .5 && Math.abs(controller.getRightX()) < .5) {
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
	public double getPOVAngle(XboxController controller, Drivetrain drivetrain) {
		double input = controller.getPOV();
		double result;
		if (input != -1) {
			if (input > 180) {
				result = input - 360;
			}
			else {
				result = input;
			}
		} else {
			result = drivetrain.lastAngle;
		}
		drivetrain.lastAngle = result;
		return result - 90;
	}
    public boolean isPOVActive(XboxController controller) {
        return controller.getPOV() != -1;
    }
    public double getRotationSetpoint(XboxController controller, Drivetrain drivetrain) {
        if (isPOVActive(controller)) {
            return getPOVAngle(controller, drivetrain);
        } else {
            return getControllerAngle(controller, drivetrain);
        }
    }
}
