// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.Stations;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoWallFlower extends SequentialCommandGroup {
  Drivetrain drive;
  /** If we are allied with a robot that wants to do it all, 
   *    we can just shoot our preloaded note and then move out of the way.
   *  A good place to move to prepare for Teleop would be the field centerline.
   *  If the robot is initially positioned at the side of the Speaker near the Amp, 
   *     there is an unobstructed path to midfield.
   *  Otherwise a slightly more complicated trajectory is necessary.
   *  This command presumes we have shot and waited some appropriate time; it just drives.
   */
  public AutoWallFlower(Drivetrain drive, Stations station) {
    this.drive = drive;
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    if ((station == Stations.Right && ally.get() == Alliance.Red) ||
        (station == Stations.Left && ally.get() == Alliance.Blue)) {
        addCommands(
	    //new DriveGeneric(drive, (xto - xfrom), (yto - yfrom))
	      new DriveGeneric(drive, FieldConstants.Halflength-RobotConstants.robotLength*1.5 -
                                            (station.x+RobotConstants.robotLength*.5*Math.cos(Math.toRadians(station.heading)))
                                    , 0.)
	      );
    } else if (station == Stations.Center) {
      if (ally.get() == Alliance.Red) {
        addCommands(
              new DriveGeneric(drive, Units.inchesToMeters(6.)
                                    , FieldConstants.Speaker2StageY + FieldConstants.StageWidth*.5 + RobotConstants.robotLength*1.5),
              new DriveGeneric(drive, FieldConstants.Halflength-RobotConstants.robotLength*1.5 -
                                            (station.x+RobotConstants.robotLength+Units.inchesToMeters(6.))
                                    , 0.)
	      );
      }else{
        addCommands(
              new DriveGeneric(drive, Units.inchesToMeters(6.), 
                                      -FieldConstants.Speaker2StageY - FieldConstants.StageWidth*.5 - RobotConstants.robotLength*1.5),
              new DriveGeneric(drive, FieldConstants.Halflength-RobotConstants.robotLength*1.5 -
                                      station.x+RobotConstants.robotLength+Units.inchesToMeters(6.)
                                    , 0.)
	      );
      }
    } else {   // from the side nearer the Source
      if (ally.get() == Alliance.Red) {
        addCommands(
	      new DriveGeneric(drive, 0.
                                    , FieldConstants.Speaker2StageY + FieldConstants.StageWidth*.5 + RobotConstants.robotLength*1.5
                                             - (station.y+RobotConstants.robotLength*.5)),
              new DriveGeneric(drive, FieldConstants.Halflength-RobotConstants.robotLength*1.5 -
                                            (station.y+RobotConstants.robotLength*.5)
                                    , 0.)
	      );
      }else{
        addCommands(
	      new DriveGeneric(drive, 0.
                                    , -FieldConstants.Speaker2StageY - FieldConstants.StageWidth*.5 - RobotConstants.robotLength*1.5
                                             + (station.y+RobotConstants.robotLength*.5)),
	      new DriveGeneric(drive, FieldConstants.Halflength-RobotConstants.robotLength*1.5 -
                                            (station.y+RobotConstants.robotLength*.5)
                                    , 0.)
	      );
      }
    }
        
  }
}
