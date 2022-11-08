package com.team498.o2022.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team498.o2022.auto.modes.*;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH_AUTO,
        ONE_BALL_LEFT_AUTO,
        ONE_BALL_RIGHT_AUTO,
        TWO_BALL_AUTO,
        TWO_BY_ONE_AUTO,
        TWO_BY_TWO_AUTO,
        FIVE_BALL_AUTO,
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Path Mode", DesiredMode.TEST_PATH_AUTO);
        mModeChooser.addOption("One Ball Left Mode", DesiredMode.ONE_BALL_LEFT_AUTO);
        mModeChooser.addOption("One Ball Right Mode", DesiredMode.ONE_BALL_RIGHT_AUTO);
        mModeChooser.addOption("Two Ball Mode", DesiredMode.TWO_BALL_AUTO);
        mModeChooser.addOption("Two by One Mode", DesiredMode.TWO_BY_ONE_AUTO);
        mModeChooser.addOption("Two by Two Mode", DesiredMode.TWO_BY_TWO_AUTO);
        mModeChooser.addOption("Five Ball Mode", DesiredMode.FIVE_BALL_AUTO);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());

        case TEST_PATH_AUTO:
            return Optional.of(new TestPathMode());
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
