package org.team498.C2022;

import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {
    private final Vision vision;
    private final Drivetrain drivetrain;
    public RobotState(Vision vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }
    public Transform2d getRobotToField() {
        return toTransform2d(drivetrain.getPose());
    }
    public Transform2d getRobotToTarget() {
        return getVisionToTarget().plus(getVisionToRobot().inverse());
    }
    public Transform2d getVisionToTarget() {
        return toTransform2d(vision.getPose());
    }
    public Transform2d getVisionToRobot() {
        return new Transform2d();
    }
    public Transform2d getVisionToField() {
        return getRobotToField().plus(getVisionToRobot());
    }
    public Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(new Pose2d(), pose);
    }
    public Pose2d toPose2d(Transform2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation());
    }
    @Override
    public void periodic() {
        drivetrain.resetOdometry(toPose2d(getRobotToField()));
    }
}
