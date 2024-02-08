// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTags extends SubsystemBase {
    private PhotonCamera camera;
  private PhotonPipelineResult result;
  /** Creates a new AprilTags. */
  public AprilTags() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }



  @Override
  public void periodic() {
    //SmartDashboard.put("April Tag X", result.getTargets());
    // This method will be called once per scheduler run
  }
}
