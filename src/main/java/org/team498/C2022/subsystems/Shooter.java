package org.team498.C2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team498.C2022.RobotState;
import org.team498.lib.util.LinearInterpolator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private RobotState robotState = RobotState.getInstance();
    private PIDController controller = new PIDController(0, 0, 0);
    private TalonFX motor = new TalonFX(0);
    private LinearInterpolator interpolator = new LinearInterpolator(null);
    private double speed = 0;

    private State state = State.VISION;

    public enum State {
        VISION(false),
        MANUAL(true);

        private boolean isManual;

        private State(boolean isManual) {
            this.isManual = isManual;
        }
    }

    @Override
    public void periodic() {
        motor.set(ControlMode.PercentOutput, controller.calculate(motor.getSelectedSensorVelocity()));
    }

    private Shooter() {
        controller.setSetpoint(speed);
    }
    
    public synchronized void setState(State state) {
        this.state = state;
    }

    public synchronized void setVisionSpeed(double distanceOffset) {
        speed = interpolator.getInterpolatedValue(robotState.getRobotToTargetDistance() + distanceOffset);
    }
    public synchronized void setVisionSpeed() {
        speed = interpolator.getInterpolatedValue(robotState.getRobotToTargetDistance());
    }
}
