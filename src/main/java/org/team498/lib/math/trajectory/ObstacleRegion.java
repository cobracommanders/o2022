package org.team498.lib.math.trajectory;

import java.util.List;

import org.team498.lib.util.TrajectoryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class ObstacleRegion {
    /**
     * Recalculates a {@link Trajectory} to avoid a set of RectangularObstacleRegions
     * @param trajectory trajectory to recalculate
     * @param regions {@link RectangularObstacleRegion regions} to avoid
     * @return new {@link Trajectory} that avoids listed regions
     */
    public static Trajectory recalculateTrajectory(Trajectory trajectory, List<? extends ObstacleRegion> regions) {
        Trajectory result = TrajectoryUtil.generateTrajectory(trajectory.features); //create new trajectory so as to avoid conflicting references
        //TODO: See if this is actually how super classes work
        for (ObstacleRegion region : regions) { //I'm not sure if using ObstacleRegion here will cause problem, but it will likely be okay
            if (!region.isOnTrajectory(trajectory)) { //if region is not on trajectory, no need to avoid
                regions.remove(region);
            }
        }
        for (ObstacleRegion region : regions) { //for each region, calculate a new trajectory to avoid the region
            result = region.calculateTrajectory(trajectory);
        }
        return result;
    }
    public abstract Trajectory calculateTrajectory(Trajectory trajectory);
    public abstract Translation2d[] findAvoidanceWaypoints(Trajectory trajectory);
    /**
    * Checks if trajectory overlaps the region
    * @param trajectory {@link Trajectory} to check
    * @return {@code true} if trajectory overlaps the region
    */
   public boolean isOnTrajectory(Trajectory trajectory) {
       for (Trajectory.State state : trajectory.getStates()) {
           if (isInRegion(state.poseMeters)) {
               return true;
           }
       }
       return false;
   }
   boolean isInRegion(Pose2d pose) {
       return isInRegion(pose.getX(), pose.getY());
   }
   abstract boolean isInRegion(double x, double y);

}
