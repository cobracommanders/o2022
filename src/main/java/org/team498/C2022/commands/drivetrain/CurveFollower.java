package org.team498.C2022.commands.drivetrain;

import org.team498.C2022.Constants;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.lib.logging.CSVReader;
import org.team498.lib.util.DriveInterpolator;
//import org.team498.lib.util.LinearInterpolator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CurveFollower extends CommandBase {
	private Drivetrain drivetrain;
	private Timer timer = new Timer();
	private DriveInterpolator interpolator;
	private double time = 0;

	public CurveFollower(Drivetrain drivetrain, double[][] trajectory) {
        //reader = new CSVReader(Constants.kRoborioTrajectoryFilepath, filename);
        //tTracker  = getArray();
		interpolator = new DriveInterpolator(trajectory);
		this.drivetrain = drivetrain;

		addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		time = timer.get();
        //double t = tInterpolator.getInterpolatedValue(time);
		double[] state = interpolator.getInterpolatedValue(time);

		drivetrain.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
                    state[1],
                    state[2],
                    state[3],
					Rotation2d.fromDegrees(drivetrain.getYaw180())
                )
        );
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
		timer.stop();
		timer.reset();
		interpolator.lastIndex = -1;
		interpolator.initialized = false;
		interpolator.finished = false;
	}

	@Override
	public boolean isFinished() {
		SmartDashboard.putBoolean("finished", interpolator.isFinished());

		return interpolator.isFinished();
	}
    // private double[][] getArray() {
	// 	reader.open();

	// 	int lines = 0;
	// 	while (reader.hasNextLine()) {
	// 		reader.nextLine();
	// 		lines++;
	// 	}
	// 	reader.close();

	// 	double[][] converted = new double[lines][];
	// 	reader.open();
	// 	for (int i = 0; i < converted.length; i++) {
	// 		String[] nextLine = reader.nextLine();
	// 		double[] newLine = new double[nextLine.length];

	// 		for (int k = 0; k < nextLine.length; k++) {
	// 			newLine[k] = Double.valueOf(nextLine[k]);
	// 		}
	// 		converted[i] = newLine;
	// 	}

	// 	return converted;
	// }
}