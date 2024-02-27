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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;

public class AutoShootAndMove extends SequentialCommandGroup {
  /** Shoots into the speaker then drives two meters in the x direction */
  public AutoShootAndMove(Arm arm, Drivetrain drive, Handler handler) {

    addCommands(Commands.race(
                            new ArmRun(arm, ArmConstants.kElbowHighSpeaker, ArmConstants.kWristHighSpeaker, 2.5),
                            new WaitCommand(2.5)), 
                            new InstantCommand(()-> arm.rearmArm()),
                          
                          Commands.race(
                            new Speaker(handler),
                            new WaitCommand(3.5)
                          ),
                          Commands.race(
                            new DriveGeneric(drive, -2, 0),
                            new WaitCommand(2)
                          )
    );



  }
}
