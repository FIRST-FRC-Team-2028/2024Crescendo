// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Stations;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;


public class ShootMovePickup extends SequentialCommandGroup {
  Handler handler;
  Drivetrain drivetrain;
  Arm arm;
  /** Shoots note in speaker from selected station,
   *  move to near note,
   *  and picks up a note. */
  public ShootMovePickup(Handler handler, Drivetrain drivetrain, Arm arm, Stations station) {
    double xdist, ydist;
    if (station == Stations.Center) {
      xdist = Constants.FieldConstants.StageX-Constants.FieldConstants.noteRadius -
                      (Constants.FieldConstants.SpeakerfaceX+Constants.RobotConstants.robotLength+Constants.RobotConstants.handlerThickness);
      ydist = 0.;
    }
    else if (station ==Stations.Left){
      xdist =0; //Constants.FieldConstants.StageX-Constants.FieldConstants.noteRadius -
                 //     (Constants.FieldConstants.SpeakerfaceX+Constants.RobotConstants.robotLength+Constants.RobotConstants.handlerThickness);
      ydist = 0.;
    }
    else {xdist = 0;
    ydist=0.;}
    
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new AutoShootAndMove(arm, drivetrain, handler, 
                      xdist, 
                    ydist),   // TODO  are these distances in the correct units?
                new ArmRun(arm, Constants.ArmConstants.kElbowFloor, Constants.ArmConstants.kWristFloor),
                new InHandler(handler),
                new Spit_Back(handler),
                Commands.race(
                new TravelPosition(arm))
                );
                // TODO use Spit_back? move arm back up?
  }

  /** Shoot note in speaker ,
   *  moves from central position to note straight behind it,
   *  pick up a note.
   */
  public ShootMovePickup(Handler handler, Drivetrain drivetrain, Arm arm) {
    this(handler, drivetrain, arm, Stations.Center);
  }
}
