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
    private final XboxController controller;
    private final Drivetrain drivetrain;
    public DriverSet set = new DriverSet();

    class DriverSet {
        DoubleSupplier x;
        DoubleSupplier y;
        DoubleSupplier rotationO;
        DoubleSupplier rotationD;
        JoystickButton resetPose;
        JoystickButton controlSet;
        JoystickButton slowDrive;
        Intake.State intakeState;
    }

    public DriverController(XboxController controller, Drivetrain drivetrain) {
        super(controller);
        this.controller = controller;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        super.periodic();
        updateDriverSet();
    }
    private void updateDriverSet() {
        set.x = ()-> squareInput(-super.set.leftY);
        set.y = ()-> squareInput(-super.set.leftX);
        set.rotationO = ()-> updateRotationSetpoint(controller, drivetrain);
        set.rotationD = ()-> squareInput(super.set.rightX);
        set.resetPose = super.set.aButton;
        set.controlSet = super.set.leftBumper;
        set.slowDrive = super.set.rightBumper;
        set.intakeState = updateIntakeState();
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
    public Intake.State getIntakeState() {
        return set.intakeState;
    }

    private double squareInput(double input) {
        return Math.copySign(Math.pow(input, 2), input);
    }
    private Intake.State updateIntakeState() {
        if (super.set.leftTrigger) {
            return State.INTAKE;
        } 
        else if (super.set.rightTrigger) {
            return State.OUTTAKE;
        }
        else {
            return State.IDLE;
        }
    }
    private double updateRotationSetpoint(XboxController controller, Drivetrain drivetrain) {
        if (isPOVActive()) {
            return getPOVAngle(drivetrain);
        } else {
            return getControllerAngle(drivetrain);
        }
    }
	private double getControllerAngle(Drivetrain drivetrain) {

		double inputY = (Math.abs(super.set.rightY) > .1) ? super.set.rightY : 0.0001;
		double inputX = (Math.abs(super.set.rightX) > .1) ? -super.set.rightX : 0.0001;
		if (Math.abs(super.set.rightY) < .5 && Math.abs(super.set.rightX) < .5) {
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
		double input = super.set.pov;
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
    private boolean isPOVActive() {
        return super.set.pov != -1;
    }
}
