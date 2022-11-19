package org.team498.lib.math.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team498.lib.util.TrajectoryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class RectangularObstacleRegion {
    final Pose2d bottomLeftPoint;
    final Pose2d bottomRightPoint;
    final Pose2d topRightPoint;
    final Pose2d topLeftPoint;
    public RectangularObstacleRegion(Pose2d bottomLeftPoint, Pose2d topRightPoint) {
        this.bottomLeftPoint = bottomLeftPoint;
        this.topRightPoint = topRightPoint;
        this.bottomRightPoint = new Pose2d(topRightPoint.getX(), bottomLeftPoint.getY(), new Rotation2d());
        this.topLeftPoint = new Pose2d(bottomLeftPoint.getX(), topRightPoint.getY(), new Rotation2d());
    }
    /**
     * Recalculates a Trajectory to avoid a set of RectangularObstacleRegions
     * @param trajectory trajectory to recalculate
     * @param regions regions to avoid
     * @return new Trajectory that avoids listed regions
     */
    public static Trajectory recalculateTrajectory(Trajectory trajectory, List<RectangularObstacleRegion> regions) {
        Trajectory result = TrajectoryUtil.generateTrajectory(trajectory.features); //create new trajectory so as to avoid conflicting references
        for (RectangularObstacleRegion region : regions) {
            if (!region.isOnTrajectory(trajectory)) { //if region is not on trajectory, no need to avoid
                regions.remove(region);
            }
        }
        for (RectangularObstacleRegion region : regions) { //for each region, calculate a new trajectory to avoid the region
            result = region.calculateTrajectory(trajectory);
        }
        return result;
    }
    /**
     * calculates Trajectory to avoid this region
     * @param trajectory Trajectory to modify
     * @return new Trajectory that avoids this region
     * @apiNote use {@code recalculateTrajectory()} for most applications
     * 
     */
    private Trajectory calculateTrajectory(Trajectory trajectory) {
        Pose2d initial = trajectory.features.start;
        ArrayList<Translation2d> waypoints = trajectory.features.waypoints;
        Pose2d end = trajectory.features.end;
        TrajectoryConfig config = trajectory.features.config;
        for (int i = 0; i < waypoints.size(); i++) {
            if (isInRegion(waypoints.get(i).getX(), waypoints.get(i).getY())) {
                Translation2d[] newWaypoints = findAvoidanceWaypoints(trajectory);
                waypoints.add(i++, newWaypoints[0]);
                waypoints.add(i++, newWaypoints[1]);
            }
        }
        return TrajectoryUtil.generateTrajectory(initial, end, waypoints, config);
    }
    /**
     * finds waypoints to avoid this region
     * @param trajectory Trajectory to apply region to
     * @return two corner Translation2d that will nav robot around region
     * @apiNote result is relative to the initial pose of the trajectory
     */
    private Translation2d[] findAvoidanceWaypoints(Trajectory trajectory) {
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        Translation2d brPoint; //min translation to bottom right corner
        Translation2d blPoint; //min translation to bottom left corner
        Translation2d trPoint; //min translation to top right corner
        Translation2d tlPoint; //min translation to top left corner
        Translation2d minPoint1 = null;
        Translation2d minPoint2 = null;
        Translation2d minPoint3 = null;
        double totalDist1 = 0;
        double totalDist2 = 0;
        double totalDist3 = 0;

        for (Trajectory.State state : trajectory.getStates()) {
            brPoint = new Translation2d(bottomRightPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            blPoint = new Translation2d(bottomLeftPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            trPoint = new Translation2d(topRightPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            tlPoint = new Translation2d(topLeftPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            //use an ArrayList for use with a foreach loop
            waypoints.add(brPoint);
            waypoints.add(blPoint);
            waypoints.add(trPoint);
            waypoints.add(tlPoint);
            for (Translation2d waypoint : waypoints) {
                double distance = waypoint.getDistance(new Translation2d(state.poseMeters.getX(), state.poseMeters.getY()));
                //sort corners by distance
                if (distance > totalDist3) {
                    if (distance > totalDist2) {
                        if (distance > totalDist1) {
                            minPoint2 = minPoint1;
                            totalDist2 = waypoint.getDistance(new Translation2d(trajectory.features.start.getX(), trajectory.features.start.getY()));
                            minPoint1 = waypoint;
                            totalDist1 = waypoint.getDistance(new Translation2d(trajectory.features.start.getX(), trajectory.features.start.getY()));
                        } else {
                            minPoint2 = waypoint;
                            totalDist2 = waypoint.getDistance(new Translation2d(trajectory.features.start.getX(), trajectory.features.start.getY()));
                        }
                    } else {
                        minPoint3 = waypoint;
                        totalDist3 = waypoint.getDistance(new Translation2d(trajectory.features.start.getX(), trajectory.features.start.getY()));

                    }
                }
            }
            //if closest points are opposing corners, replace furthest corner with closest adjacent corner
            if ((minPoint1 == brPoint || minPoint1 == tlPoint) && (minPoint2 == tlPoint || minPoint2 == brPoint)
            || ((minPoint1 == blPoint || minPoint1 == trPoint) && (minPoint2 == trPoint || minPoint2 == blPoint))) {
                minPoint2 = minPoint3;
            }
        }
        //TODO: currently maps new waypoints relative to initial pose-- allow for mapping on different coordinate systems
        if (totalDist1 >= totalDist2) {
            return new Translation2d[] {minPoint1, minPoint2};
        } else {
            return new Translation2d[] {minPoint2, minPoint1};
        }
        
    }
    public boolean isOnTrajectory(Trajectory trajectory) {
        for (Trajectory.State state : trajectory.getStates()) {
            if (isInRegion(state.poseMeters)) {
                return true;
            }
        }
        return false;
    }
    private boolean isInRegion(Pose2d pose) {
        return isInRegion(pose.getX(), pose.getY());
    }
    private boolean isInRegion(double x, double y) {
        return (x >= bottomLeftPoint.getX() && x <= topRightPoint.getX()
            && (y >= bottomLeftPoint.getY() && y <= topRightPoint.getY()));
    }
}
