package org.team498.C2022.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera cam1 = new PhotonCamera("gloworm");
    PhotonPipelineResult pipelineResult;
    Pose2d pose = new Pose2d();
    Field2d field = new Field2d();

    public Vision() {
        cam1.setPipelineIndex(0);
        cam1.setDriverMode(false);
        pipelineResult = cam1.getLatestResult();
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
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d());
    }
}
