package org.team498.C2022.commands.wrist;

import org.team498.C2022.subsystems.Wrist;
import org.team498.C2022.subsystems.Wrist.State;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetWrist extends CommandBase {
	private final Wrist wrist = Wrist.getInstance();
	private final State state;

	public SetWrist(State state) {
		this.state = state;
		addRequirements(wrist);
	}

	@Override
	public void execute() {
		wrist.setState(state);
	}
}
