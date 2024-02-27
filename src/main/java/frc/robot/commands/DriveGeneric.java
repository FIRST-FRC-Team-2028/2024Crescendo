// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveGeneric extends Command {
  int randomNumber = Math.floorMod(System.currentTimeMillis(), 1000);
  Drivetrain driver;
  Pose2d startpose, targetpose;
  double xdist;
  double ydist;
  double encS;
  PIDController controller;
  double target, dist;
  double tol;
  boolean stopwhendone = true;
  boolean iShouldStop = false;
  private double variableP;
  /** Drive a given distance in any direction in field coordinates.
   * 
   * @param xdist and
   * @param ydist are component distances in meters
   * @param stopwhendone  false directs the motors to continue their state when the final position is accomplished
   */
  public DriveGeneric(Drivetrain driveon, double xdist, double ydist, boolean stopwhendone) {
    addRequirements(driveon);
    this.driver = driveon;
    this.xdist = xdist;
    this.ydist = ydist;
    this.stopwhendone = stopwhendone;
    controller = new PIDController(0, 0, 0);  // set p in init
    variableP=.9;
  }

  /** Drive a given distance in any direction in field coordinates.
   * 
   * @param xdist and
   * @param ydist are component distances in meters
   */
  public DriveGeneric(Drivetrain driveon, double xdist, double ydist) {
    this(driveon, xdist, ydist, true);
  }
  /** Drive a given distance in any direction in field coordinates.
   * 
   * @param xdist and
   * @param ydist are component distances in meters
   * @param varP overrides default kP of PIDcontroller
   */
  public DriveGeneric(Drivetrain driveon, double xdist, double ydist, double varP) {
    this(driveon, xdist, ydist, true);
    this.variableP=varP;
  }

  /** stop DriveGeneric Command whether accomplished or not */
  public void endDriveGeneric() {
    iShouldStop = true;
    // MrG this should use the Drivetrain method to stop
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driver.makemefalse();
    // encS = driver.returnEncode()[2];
    dist = Math.sqrt(xdist*xdist+ydist*ydist);
    // target = encS+dist;
    tol = 0.04*dist;
    controller.setP(variableP/dist);
    startpose = driver.getPose();
    Transform2d transform = new Transform2d(new Translation2d(xdist, ydist), new Rotation2d(0));
    targetpose = startpose.plus(new Transform2d(new Translation2d(xdist, ydist), new Rotation2d(0)));
    //SmartDashboard.putString("Transform", transform.toString());
    SmartDashboard.putString("TargetPose", targetpose.toString());
    SmartDashboard.putString("StartPose", startpose.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentpose = driver.getPose();
    double whereiam = PhotonUtils.getDistanceToPose(currentpose, startpose);
    double speed = controller.calculate(whereiam, dist);
    driver.driveit(speed*xdist/dist, speed*ydist/dist, 0, true);
    System.out.println("whereiam= "+ whereiam);

    SmartDashboard.putString("Error", controller.getPositionError() + " < " + tol);
    SmartDashboard.putString("CurrentPose", currentpose.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopwhendone)
      driver.stopModules();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.getPositionError() < tol ||
    //iShouldStop;
      driver.shouldistop();
  }
}