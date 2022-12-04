package org.team498.C2022.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private static Vision mInstance;

	public static Vision getInstance() {
		if (mInstance == null) {
			mInstance = new Vision();
		}
		return mInstance;
	}
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    PhotonCamera cam1 = new PhotonCamera("gloworm");
    PhotonPipelineResult pipelineResult;
    Pose2d pose = new Pose2d();
    Field2d field = new Field2d();

    public Vision() {
        cam1.setPipelineIndex(0);
        cam1.setDriverMode(false);
    }
    @Override
    public void periodic() {
        pipelineResult = cam1.getLatestResult();
        if (pipelineResult.hasTargets()) {
            pose = getPose();
            drivetrain.resetOdometry(pose);
            field.setRobotPose(pose);
        }
        SmartDashboard.putNumber("Cam X", pose.getX());
        SmartDashboard.putData(field);
    }
    public Pose2d getPose() {
        Transform3d pose = pipelineResult.getBestTarget().getBestCameraToTarget();
        return new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(drivetrain.getYaw180()));
    }
    public Pose2d getPoseWithRotation() {
        Transform3d pose = pipelineResult.getBestTarget().getBestCameraToTarget();
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d());
    }
}
