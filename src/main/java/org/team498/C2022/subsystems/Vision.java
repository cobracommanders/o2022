package org.team498.C2022.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class Vision extends SubsystemBase {
    private final PhotonCamera cam1 = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    private PhotonPipelineResult pipelineResult;
    private Pose2d pose = new Pose2d();
    private final Field2d field = new Field2d();

    private Vision() {
        cam1.setPipelineIndex(0);
        cam1.setDriverMode(false);
    }

    @Override
    public void periodic() {
        pipelineResult = cam1.getLatestResult();
        if (pipelineResult.hasTargets()) {
            pose = getPose();
            field.setRobotPose(pose);
        }
        SmartDashboard.putNumber("Cam X", pose.getX());
        SmartDashboard.putData(field);
    }

    public Pose2d getPose() {
        Transform3d pose = pipelineResult.getBestTarget().getBestCameraToTarget();
        double y = -pose.getX() / Math.tan(Math.PI / 2 - pose.getRotation().getZ());
        return new Pose2d(pose.getX(), y, new Rotation2d(-pose.getRotation().toRotation2d().getRadians()));
    }


    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}
