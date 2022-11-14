package org.team498.lib.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryUtil {
    public static Trajectory getTrajectory(String fileName) {
        Trajectory trajectory = new Trajectory();
		Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
		try {
			trajectory = edu.wpi.first.math.trajectory.TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException e) {
			e.printStackTrace();
		}
        return trajectory;
	}
    public static Trajectory generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> waypoints) {
        waypoints.add(new Translation2d(1, 1));
        TrajectoryConfig config = new TrajectoryConfig(2, 3);
        config.setReversed(true);
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }
}
