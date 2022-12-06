package org.team498.C2022.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2022.subsystems.Drivetrain;

public class LockWheels extends CommandBase {
    private final Drivetrain drivetrain;
    private final SwerveModuleState[] lockedStates = new SwerveModuleState[] {
            // Front left
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            // Front right
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            // Back left
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            // Back right
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))};

    public LockWheels() {
        this.drivetrain = Drivetrain.getInstance();
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setModuleStates(lockedStates, true);
    }
}
