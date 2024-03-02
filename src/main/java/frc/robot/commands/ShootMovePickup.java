// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;


public class ShootMovePickup extends SequentialCommandGroup {
  Handler handler;
  Drivetrain drivetrain;
  Arm arm;
  AutoShootAndMove autoShootAndMove;
  /** Shoots note in speaker,
   *  moves from central position to note straight behind it,
   *  and picks up a note. */
  public ShootMovePickup(Handler handler, Drivetrain drivetrain, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new AutoShootAndMove(arm, drivetrain, handler, 
       Constants.FieldConstants.StageX-Constants.FieldConstants.noteRadius-(Constants.FieldConstants.SpeakerfaceX+Constants.RobotConstants.robotLength+Constants.RobotConstants.handlerThickness), 
       0.), 
       new ArmRun(arm, Constants.ArmConstants.kElbowFloor, Constants.ArmConstants.kWristFloor),
       new InHandler(handler));
    

  }

  
}
