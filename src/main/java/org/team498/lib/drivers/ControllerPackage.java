package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerPackage extends SubsystemBase {
    final XboxController controller;
    public ControllerSet set;
    public class ControllerSet {
        public double leftX;
        public double leftY;
        public double rightX;
        public double rightY;
        public int pov;
        public JoystickButton aButton;
        public JoystickButton bButton;
        public JoystickButton xButton;
        public JoystickButton yButton;
        public JoystickButton startButton;
        public JoystickButton backButton;
        public JoystickButton rightBumper;
        public JoystickButton leftBumper;
        public Trigger leftTrigger;
        public Trigger rightTrigger;
    }
    public ControllerPackage(XboxController controller) {
        this.controller = controller;
    }
    private void updateControllerSet() {
        set.leftY = controller.getLeftY();
        set.leftX = controller.getLeftX();
        set.rightX = controller.getRightX();
        set.rightY = controller.getRightY();
        set.pov = controller.getPOV();
        set.aButton = new JoystickButton(controller, Button.kA.value);
        set.bButton = new JoystickButton(controller, Button.kB.value);
        set.xButton = new JoystickButton(controller, Button.kX.value);
        set.yButton = new JoystickButton(controller, Button.kY.value);
        set.rightBumper = new JoystickButton(controller, Button.kRightBumper.value);
        set.leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
        set.rightTrigger = new Trigger(()-> Math.abs(controller.getRightTriggerAxis()) >= 0.3);
        set.leftTrigger = new Trigger(()-> Math.abs(controller.getLeftTriggerAxis()) >= 0.3);
    }
    @Override
    public void periodic() {
        updateControllerSet();
    }
}
