package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControllerPackage extends SubsystemBase {
    final XboxController controller;
	public double leftX = 0;
	public double leftY = 0;
	public double rightX = 0;
	public double rightY = 0;
	public int pov = -1;
	public final JoystickButton aButton;
	public final JoystickButton bButton;
	public final JoystickButton xButton;
	public final JoystickButton yButton;
	public final JoystickButton startButton;
	public final JoystickButton backButton;
	public final JoystickButton rightBumper;
	public final JoystickButton leftBumper;
	public boolean leftTrigger;
	public boolean rightTrigger;

    public ControllerPackage(XboxController controller) {
        this.controller = controller;
		leftX = 0;
		leftY = 0;
		rightX = 0;
		rightY = 0;
		pov = -1;
		aButton = new JoystickButton(controller, Button.kA.value);
		bButton = new JoystickButton(controller, Button.kB.value);
		xButton = new JoystickButton(controller, Button.kX.value);
		yButton = new JoystickButton(controller, Button.kY.value);
		startButton = new JoystickButton(controller, Button.kStart.value);
		backButton = new JoystickButton(controller, Button.kBack.value);
		rightBumper = new JoystickButton(controller, Button.kRightBumper.value);
		leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
		leftTrigger = Math.abs(controller.getRightTriggerAxis()) >= 0.3;
		rightTrigger = Math.abs(controller.getLeftTriggerAxis()) >= 0.3;
    }



    private void updateControllerSet() {
        leftY = controller.getLeftY();
        leftX = controller.getLeftX();
        rightX = controller.getRightX();
        rightY = controller.getRightY();
        pov = controller.getPOV();
        //set.aButton = new JoystickButton(controller, Button.kA.value);
        //set.bButton = new JoystickButton(controller, Button.kB.value);
        //set.xButton = new JoystickButton(controller, Button.kX.value);
        //set.yButton = new JoystickButton(controller, Button.kY.value);
        //set.rightBumper = new JoystickButton(controller, Button.kRightBumper.value);
        //set.leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
        rightTrigger = Math.abs(controller.getRightTriggerAxis()) >= 0.3;
        leftTrigger = Math.abs(controller.getLeftTriggerAxis()) >= 0.3;
    }
    @Override
    public void periodic() {
        updateControllerSet();
    }
}
