// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilCamera extends SubsystemBase {
  private PhotonCamera camera;
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget target;
  Pose3d robotPose;
  AprilTagFieldLayout aprilTagFieldLayout;
  double distanceToTarget;
  Transform3d robotToCam;
  PhotonPoseEstimator photonPoseEstimator;
  //private PhotonPipelineResult result;
  /** Creates a new AprilTags. */
  public AprilCamera() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
     //Cam mounted facing forward, 0.3302 meters in front of the center, 0 meters left/right of center, 
     // and 0.1778 meters of elevation (off floor)            on project X
    robotToCam = new Transform3d(new Translation3d(0.3302, 0.0, 0.1778),
                new Rotation3d(0,0,0));
 
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                        PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);

  }



  public Pose3d getRobotPosition() {
    return robotPose;
  }
  public void print() {
    System.out.println(getRobotPosition());
  }

  public void showYaw() {
    SmartDashboard.putNumber("YE Yaw", target.getYaw());
  }



  @Override
  public void periodic() {
    showYaw();
    photonPoseEstimator.update();
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    targets = result.getTargets();
    target = result.getBestTarget();
    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
              aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);
    //SmartDashboard.put("April Tag X", result.getTargets());
    // This method will be called once per scheduler run
  }
}
