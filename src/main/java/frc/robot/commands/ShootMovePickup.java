// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
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
    double xdist, ydist, heading;
    if (station == Stations.Center) {
      xdist = Constants.FieldConstants.StageX-Constants.FieldConstants.noteRadius -
                      (Constants.FieldConstants.SpeakerfaceX+Constants.RobotConstants.robotLength+Constants.RobotConstants.handlerThickness);
      ydist = 0.;
      heading = 0;
    }
    else if (station ==Stations.Left){
      xdist = Constants.FieldConstants.StageX-Constants.FieldConstants.noteRadius-Constants.RobotConstants.robotLength*.5 -
                       (Constants.Stations.Left.x +
                        Constants.RobotConstants.robotLength*.5*Math.cos(Math.toRadians(Stations.Left.heading)));
      ydist = Constants.FieldConstants.StageY+FieldConstants.noteDistance - 
                       (Constants.Stations.Left.y+Constants.RobotConstants.robotLength*.5*Math.sin(Math.toRadians(60.)));
      heading = Stations.Left.heading;
    }else {  // station Right
      xdist = Constants.FieldConstants.StageX-Constants.FieldConstants.noteRadius-Constants.RobotConstants.robotLength*.5 -
                       (Constants.Stations.Right.x +
                        Constants.RobotConstants.robotLength*.5*Math.cos(Math.toRadians(Stations.Right.heading)));
      ydist = Constants.FieldConstants.StageY-FieldConstants.noteDistance - 
                       (Constants.Stations.Right.y-Constants.RobotConstants.robotLength*.5*Math.sin(Math.toRadians(60.)));
      heading = Stations.Right.heading;                  
    }
    
    // Use addRequirements() here to declare subsystem dependencies.
    System.out.println("xdist, ydist = "+xdist+" "+ydist);
    addCommands(new AutoShootAndMove(arm, drivetrain, handler, 
                      xdist, 
                    ydist),   // TODO  are these distances in the correct units?
                  Commands.parallel(
                    Commands.race(
                      new InHandler(handler),
                      new WaitCommand(1) //can't go in forever
                    ).andThen(new Spit_Back(handler)),
                    Commands.race(
                      new WaitCommand(1),
                      new DriveGeneric(drivetrain, 0.5, 0)
                    )
                  ),
                  Commands.parallel(
                    new DriveGenericHead(drivetrain, -xdist, -ydist, heading),
                    new TravelPosition(arm).
                    andThen(new ArmRun(arm, ArmConstants.kElbowSpeaker, ArmConstants.kWristSpeaker)).
                    andThen(new InstantCommand(()-> arm.rearmArm()))
                  ),
                  new Speaker(handler)
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
