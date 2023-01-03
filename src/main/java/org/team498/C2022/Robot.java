package org.team498.C2022;

import org.team498.C2022.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer = RobotContainer.getInstance();
	private Command autoCommand;

	@Override
	public void robotInit() {
		robotContainer.getRobotInitCommand().schedule();
		Drivetrain.getInstance().setGyroOffset(0);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autoCommand = robotContainer.getAutoCommand();

		if (autoCommand != null) {
			autoCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
	}

}
