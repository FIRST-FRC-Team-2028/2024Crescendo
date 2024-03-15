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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CamConstant;
import frc.robot.Constants.Lights;

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
  private Drivetrain drivetrain;
  private final Solenoid blue;
  
  //private PhotonPipelineResult result;
  /** Creates a new AprilTags. */
  public AprilCamera() {

    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    blue = new Solenoid(PneumaticsModuleType.CTREPCM, Lights.blue); //April tags
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
     //Cam mounted facing forward, 0.3302 meters in front of the center, 0 meters left/right of center, 
     // and 0.1778 meters of elevation (off floor)            on project X
    robotToCam = new Transform3d(new Translation3d(0.3302, 0.0, 0.1778),
                new Rotation3d(0,0,0));
 
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                        PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);

  }


  public void aprilTagsOn() {
    blue.set(true);
  }

  public void aprilTagsOff() {
    blue.set(false);
  }


  public Pose3d getRobotPosition() {
    return robotPose;
  }
  public void print() {
    System.out.println(getRobotPosition());
  }
  public PhotonPoseEstimator camPose() {
    return photonPoseEstimator;
  }
  public double tagYaw() {
    return target.getYaw();
  }
  public boolean target() {
    return hasTargets && (target.getFiducialId() == Constants.CamConstant.redSpeaker || target.getFiducialId() == CamConstant.blueSpeaker);
  }

  //public void showYaw() {
  //  SmartDashboard.putNumber("YE Yaw", target.getYaw());         IF DONT HAVE TARGET, DONT RUN SHOWYAW
  //}



  @Override
  public void periodic() {

    //photonPoseEstimator.update();
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets) {
    targets = result.getTargets();
    target = result.getBestTarget();

    //robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
    //          aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);
    //showYaw();

    //SmartDashboard.putString("Robot Pose", photonPoseEstimator.toString());
    SmartDashboard.putNumber("April Tag X", target.getFiducialId());
    SmartDashboard.putNumber("Get Yaw", target.getYaw());
    // This method will be called once per scheduler run
    }
  }
}
