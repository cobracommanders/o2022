package org.team498.C2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team498.C2022.Constants.WristConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
	private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, new TrapezoidProfile.Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));

	private CANSparkMax leftMotor = new CANSparkMax(WristConstants.leftID, MotorType.kBrushless);
	private CANSparkMax rightMotor = new CANSparkMax(WristConstants.rightID, MotorType.kBrushless);
	private RelativeEncoder encoder = rightMotor.getEncoder();


	private State state = State.IN;

    public enum State {
        IN(WristConstants.kInPosition),
        OUT(WristConstants.kOutPosition);

        private double position;
        private State(double position) {
            this.position = position;
        }
    }
	public Wrist() {
		rightMotor.restoreFactoryDefaults();
		leftMotor.restoreFactoryDefaults();
		rightMotor.follow(leftMotor, true);
		controller.setTolerance(.15, .1);
	}

	@Override
	public void periodic() {
		TrapezoidProfile.State goal = new TrapezoidProfile.State(state.position, 0);
		double output = controller.calculate(encoder.getPosition(), goal);
		leftMotor.set(output);
		SmartDashboard.putNumber("wrist pose", output);
		SmartDashboard.putNumber("wrist encoder", encoder.getPosition());
	}

	public State getState() {
		return this.state;
	}

	public void setState(State state) {
		this.state = state;
	}
}
