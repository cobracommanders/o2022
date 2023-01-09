package org.team498.C2022;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team498.C2022.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    DoubleSupplier x;
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        x = robotContainer.test();

        // Calibrate the gyro sensor when the robot is powered on
        Drivetrain.getInstance().calibrateGyro();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        System.out.println(x.getAsDouble());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}
