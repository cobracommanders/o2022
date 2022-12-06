package org.team498.lib.math.trajectory;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class Trajectory extends edu.wpi.first.math.trajectory.Trajectory {
    public PathFeatures features;
    public static class PathFeatures {
        public final Pose2d start;
        public final Pose2d end;
        public ArrayList<Translation2d> waypoints;
        public final TrajectoryConfig config;
        public PathFeatures(Pose2d start, Pose2d end, ArrayList<Translation2d> waypoints, TrajectoryConfig config) {
            this.start = start;
            this.end = end;
            this.waypoints = waypoints;
            this.config = config;
        }
    }
    public Trajectory() {
        //this.features = new PathFeatures(super.getInitialPose(), super.getStates().get(super.getStates().size() - 1), super.ge, config)

    }
}
