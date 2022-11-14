package com.team498.o2022.subsystems;

import com.team498.o2022.RobotState;
import com.team498.o2022.controlboard.ControlBoard;
import com.team498.o2022.drivers.AHRS;
import com.team498.o2022.logger.LogStorage;
import com.team498.o2022.logger.LoggingSystem;
import com.team498.o2022.loops.ILooper;
import com.team498.o2022.loops.Loop;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class Superstructure extends Subsystem {

    // superstructure instance
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    /*** REQUIRED INSTANCES ***/
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final AHRS mGyro = AHRS.getInstance();

    // robot state
    private final RobotState mRobotState = RobotState.getInstance();

    // PeriodicIO instance and paired csv writer
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    /*** CONTAINER FOR SUPERSTRUCTURE ACTIONS AND GOALS ***/
    public static class PeriodicIO {
        // time measurements
        public double timestamp;
        public double dt;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();

                // send log data
                SendLog();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    /***
     * CONTAINER FOR OPERATOR COMMANDS CALLING SUPERSTRUCTURE ACTIONS
     * 
     * Intaking
     * - hold right trigger to intake
     * - hold left trigger to manual eject
     * 
     * Shooting
     * - press A to prep for shot (spin up)
     * - press Y to shoot once ready
     * - press B to toggle fender shot with set params
     * - press X to toggle spit shot with set params
     * 
     * Manual Hood Adjustment
     * - Use dpad to manually adjust hood with offset
     * --> 0 to move hood up
     * --> 180 to move hood down
     * - press START button to reset adjustment
     * 
     * Other Manual Sets
     * - press dpad left (POV 270) to toggle force intake
     * - press dpad right (POV 90) to toggle disabling the ejector
     * - hold left bumper to eject balls manuallyo
     * 
     * Climb Controls
     * - press left bumper, right bumper, left trigger, right trigger to enter climb
     * mode
     * - press center "back" and "start" buttons to exit climb mode
     * - press down on left joystick to toggle open loop control of climber arms
     * - press down on right joystick to zero position on climber motors
     * 
     * - press A to prep for climb and extend right arm
     * - press B to only climb to mid bar
     * - press Y to complete full automated climb to traversal bar
     * 
     * - DPad down (POV 180) to climb mid bar and extend for high bar
     * - DPad right (POV 90) to climb high bar and extend for traversal bar
     * - DPad up (POV 0) to climb traversal bar
     * 
     */
    public void updateOperatorCommands() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
    }

    /* Initial states for superstructure for teleop */
    public void setInitialTeleopStates() {
        System.out.println("Set initial teleop states!");
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "SUPERSTRUCTURE_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");
        headers.add("dt");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(mPeriodicIO.timestamp);
        items.add(mPeriodicIO.dt);

        // send data to logging storage
        mStorage.addData(items);
    }

    public boolean isAimed() {
        return false;
    }

    public boolean hasTarget() {
        return false;
    }

}
