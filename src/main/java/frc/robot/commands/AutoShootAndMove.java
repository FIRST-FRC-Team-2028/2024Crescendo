// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;

public class AutoShootAndMove extends SequentialCommandGroup {
  /** Shoots into the speaker then drives 
   * @param xdist, 
   * @param ydist meters in the x direction 
   */
  public AutoShootAndMove(Arm arm, Drivetrain drive, Handler handler, AprilCamera aprilCamera, double xdist, double ydist) {

    addCommands(
                      Commands.parallel(
                            new ArmRun(arm, arm.getElbowPos(), ArmConstants.kWristHighSpeaker, 2.5),
                            new InstantCommand(()-> handler.high_out())
                      ),
                      
                new InstantCommand(()-> arm.rearmArm()),
                          
                
                new WaitCommand(1).
                  andThen(new Speaker(handler, aprilCamera)).
                    andThen(new ArmRun(arm, arm.getElbowPos(), ArmConstants.wristTravel-10, 3.)).
                      andThen(new TravelPosition(arm)).
                        andThen(new InstantCommand(()-> arm.rearmArm())),
                      
                          
                Commands.parallel(
                      new WaitCommand(5).
                        andThen(new DriveGenericHead(drive, xdist, ydist, DriveConstants.kRotateToZero, AutoConstants.kToNoteTurnP)),
                      new ArmRun(arm, ArmConstants.kElbowPreFloow, ArmConstants.kWristPreFloor, 2).
                        andThen(new ArmRun(arm, ArmConstants.kElbowFloor, ArmConstants.kWristFloor, 2)).
                          andThen(new InstantCommand(()-> arm.rearmArm()))
                          )
    );  // TODO this process takes 8 seconds; must be quicker to shoot two notes in auto
  }


  /** Shoots into the speaker then drives two meters in the x direction */
  public AutoShootAndMove(Arm arm, Drivetrain drivetrain, Handler handler, AprilCamera aprilCamera){
                          this(arm, drivetrain, handler, aprilCamera, 2., 0.);
  }
}
