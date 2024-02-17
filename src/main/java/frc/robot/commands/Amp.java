// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Handler;

public class Amp extends Command {
  Handler intake;
  Timer timer;

  /** Put the note in the Amp */
  public Amp(Handler Intake) {
    intake = Intake;
    timer = new Timer();
    addRequirements(intake);   //here to declare subsystem dependencies. TODO
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    intake.shootInAmpHigh();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(0.1)) {
      intake.shootInAmpLow();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
