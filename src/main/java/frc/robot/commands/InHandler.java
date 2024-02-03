// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class InHandler extends Command {
  Intake intake;
  /** Acquire a note */
  public InHandler(Intake Intake) {
    intake = Intake;
    // Use addRequirements() here to declare subsystem dependencies. TODO
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 // if (/*Color sensor reads if there is a note*/) {              COLOR SENSOR STUFF
 //   intake.iHaveIt();                                           COLOR SENSOR STUFF
 // } else intake.iDontHaveIt();                                  COLOR SENSOR STUFF
  
  if (!intake.doIHaveIt()); {
      intake.low_in();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // if (/*Color sensor reads if there is a note*/) {              COLOR SENSOR STUFF
  //   intake.iHaveIt();                                           COLOR SENSOR STUFF
  // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //COLOR SENSOR STUFF GOES HERE TO MAKE IT STOP
    //NOT SETUP FOR A STOP BUTTON
    return intake.doIHaveIt();
  }
}
