// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Wait extends Command {
  private double msecs;
  Timer timer;
  /** takes time to finish doing nothing. */
  public Wait(double msecs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.msecs = msecs;
    timer = new Timer();
    //long randNum = Math.floorMod(System.currentTimeMillis(),1000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(msecs);
  }
}