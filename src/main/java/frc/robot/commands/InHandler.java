// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj2.command.Command;
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
  if (handler.shouldIUseSensor()){
    if (handler.useSensor()) {
      handler.stop();
    }
    else {
        handler.iDontHaveIt();
        handler.low_PickUp();
    }
  } else {
    handler.iDontHaveIt();
    handler.low_PickUp();
  }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (handler.useSensor()) {
      handler.iHaveIt(); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    handler.stop();
    handler.noteOn();
    //handler.spit_Back();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return handler.doIHaveIt();

  }
}
