package com.team498.o2022.shuffleboard.tabs;

import com.team498.o2022.shuffleboard.ShuffleboardTabBase;
import com.team498.o2022.subsystems.Superstructure;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SuperstructureTab extends ShuffleboardTabBase {

	private Superstructure mSuperstructure = Superstructure.getInstance();


	// additional status vars
	private NetworkTableEntry mHasTarget;
	private NetworkTableEntry mIsAimed;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Superstructure");

		mHasTarget = mTab
				.add("Has Vision Target", false)
				.withSize(2, 1)
				.getEntry();
		mIsAimed = mTab
				.add("Is Vision Aimed", false)
				.withSize(2, 1)
				.getEntry();

	}

	@Override
	public void update() {
        // update other status vars
        mHasTarget.setBoolean(mSuperstructure.hasTarget());
        mIsAimed.setBoolean(mSuperstructure.isAimed());
	}

}
