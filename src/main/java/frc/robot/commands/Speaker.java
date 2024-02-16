// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Handler;

public class Speaker extends Command {
  Handler handler;
  Timer timer;

  /* Shoot into the Speaker:
   *   Presuming the robot is positioned and named9
   */
  /** Shoot note into Speaker */
  public Speaker(Handler handler) {
    this.handler = handler;
    timer = new Timer();
    addRequirements(handler);   // here to declare subsystem dependencies. TODO
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    handler.high_out();

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(2)) handler.low_ToHigh();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    handler.stop();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(3);
  }
}
