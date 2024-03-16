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
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;

public class AutoShoot extends SequentialCommandGroup {
  /** Shoots into the speaker then drives two meters in the x direction */
  public AutoShoot(Arm arm, Handler handler, AprilCamera aprilCamera) {

    addCommands(/*Commands.race(
                            new TravelPosition(arm),
                            new WaitCommand(3)
                          ),*/
                          
                          Commands.race(
                            new ArmRun(arm, ArmConstants.kElbowHighSpeaker, ArmConstants.kWristHighSpeaker, 2),
                            new WaitCommand(2)), 
                          new InstantCommand(() -> arm.rearmArm()),
                          Commands.race(
                            new Speaker(handler, aprilCamera),
                            new WaitCommand(3.5)
                          )
                          //new DriveGeneric(drive, 2, 0)
    );
    // TODO 5.5 seconds to shoot


  }
}
