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
    public static Trajectory recalculateTrajectory(Trajectory trajectory, List<RectangularObstacleRegion> regions) {
        for (RectangularObstacleRegion region : regions) {
            if (!region.isOnTrajectory(trajectory)) {
                regions.remove(region);
            }
        }
        for (RectangularObstacleRegion region : regions) {
            trajectory = region.calculateTrajectory(trajectory);
        }
        return trajectory;
    }

    public Trajectory calculateTrajectory(Trajectory trajectory) {
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
    private Translation2d[] findAvoidanceWaypoints(Trajectory trajectory) {
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        Translation2d brPoint; //min distance to bottom right corner
        Translation2d blPoint; //min distance to bottom left corner
        Translation2d trPoint; //min distance to top right corner
        Translation2d tlPoint; //min distance to top left corner
        Translation2d minPoint1 = null;
        Translation2d minPoint2 = null;
        double minDist_1 = 0;
        double minDist_2 = 0;

        for (Trajectory.State state : trajectory.getStates()) {
            brPoint = new Translation2d(bottomRightPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            blPoint = new Translation2d(bottomLeftPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            trPoint = new Translation2d(topRightPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            tlPoint = new Translation2d(topLeftPoint.relativeTo(state.poseMeters).getX(), bottomRightPoint.relativeTo(state.poseMeters).getY());
            
            waypoints.add(brPoint);
            waypoints.add(blPoint);
            waypoints.add(trPoint);
            waypoints.add(tlPoint);
            for (Translation2d waypoint : waypoints) {
                double distance = waypoint.getDistance(new Translation2d(state.poseMeters.getX(), state.poseMeters.getY()));
                if (distance > minDist_2) {
                    if (distance > minDist_1) {
                        minDist_1 = distance;
                        minPoint1 =  waypoint;
                    } else {
                        minDist_2 = distance;
                        minPoint2 = waypoint;
                    }
                }
            }
        }
        if (minDist_1 >= minDist_2) {
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
