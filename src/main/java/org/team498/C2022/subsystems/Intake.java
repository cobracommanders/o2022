package org.team498.C2022.subsystems;

import org.team498.C2022.Constants.IntakeContants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake mInstance;

	public static Intake getInstance() {
		if (mInstance == null) {
			mInstance = new Intake();
		}
		return mInstance;
	}
    private State state = State.IDLE;

    public enum State {
        INTAKE(IntakeContants.kIntakeSpeed),
        IDLE(IntakeContants.kIdleSpeed),
        OUTTAKE(IntakeContants.kOuttakeSpeed);


        private double speed;
        private State(double speed) {
            this.speed = speed;
        }
    }
    
    public Intake() {
        configMotor();
    }

    @Override
    public void periodic() {
        setSpeed(state.speed);
    }

    private void configMotor() {
        //TODO: complete method
    }

    public State getState() {
        return state;
    }
    public void setState(State state) {
        this.state = state;
    }
    private void setSpeed(double speed) {
        //TODO: complete method
        //motor.set(ControlMode.PercentOutput, speed);
    }
}
