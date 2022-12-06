package org.team498.C2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2022.subsystems.Drivetrain;
import org.team498.C2022.subsystems.Vision;

public class RobotState extends SubsystemBase {
    private final Vision vision;
    private final Drivetrain drivetrain;

    private RobotState() {
        this.vision = Vision.getInstance();
        this.drivetrain = Drivetrain.getInstance();
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

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
}
