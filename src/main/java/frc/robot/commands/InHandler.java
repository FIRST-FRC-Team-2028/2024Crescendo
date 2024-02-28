// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Handler;

public class InHandler extends Command {
  Handler handler;
  /** Acquire a note
   * run an handler motor in the handler direction
   * stop motor when note is acquired 
   */
  public InHandler(Handler Handler) {
    handler = Handler;
    addRequirements(handler); // here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 // if (/*Color sensor reads if there is a note*/) {              COLOR SENSOR STUFF
 //   handler.iHaveIt();                                           COLOR SENSOR STUFF
 // } else handler.iDontHaveIt();                                  COLOR SENSOR STUFF
  
     //if (handler.useSensor()) {
      //handler.stop();
    //}
    //else {
      handler.iDontHaveIt();
      handler.low_PickUp();
    //}

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (handler.useSensor()) {
      handler.iHaveIt(); 
    }*/

  // if (/*Color sensor reads if there is a note*/) {              COLOR SENSOR STUFF
  //   handler.iHaveIt();                                           COLOR SENSOR STUFF
  // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    handler.stop();
    //handler.spit_Back();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //COLOR SENSOR STUFF GOES HERE TO MAKE IT STOP
    //NOT SETUP FOR A STOP BUTTON
    return handler.doIHaveIt();
    
  }
}
