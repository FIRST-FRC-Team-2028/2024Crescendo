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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;

public class AutoDriveOut extends SequentialCommandGroup {
  /** Shoots into the speaker then drives 
   * @param xdist, 
   * @param ydist meters in the x direction 
   */
  public AutoDriveOut( Drivetrain drive){

    addCommands(
                  new DriveGeneric(drive, 3, 0)
                          );  // TODO this process takes 8 seconds; must be quicker to shoot two notes in auto
  }


 
}
