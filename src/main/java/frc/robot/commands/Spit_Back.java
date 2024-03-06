// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

public class Spit_Back extends Command {
  Handler m_handler;
  Timer time;
  /** Move note off of high speed wheels. Low Out, Wait, Stop*/
  public Spit_Back(Handler handler) {
    addRequirements(handler);
    this.m_handler = handler;
    time = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_handler.low_out();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_handler.stop();
    time.stop();
    time.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.hasElapsed(.15 )) {
      return true;}
    else 
  {return false;
    }
    }

  }
