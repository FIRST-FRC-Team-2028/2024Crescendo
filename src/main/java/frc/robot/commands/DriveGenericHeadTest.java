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

public class DriveGenericHeadTest extends SequentialCommandGroup {
  /** Creates a new DriveGenericHeadTest. */
  public DriveGenericHeadTest(Drivetrain drive, Arm arm, Handler handler) {
    addCommands(
                new DriveGenericHead(drive, -2, 0, 30, true)
    );  
  }
}
