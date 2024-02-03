// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Speaker extends Command {
  Intake intake;
  Timer timer;

  /** Shoot note into Speaker */
  public Speaker(Intake Intake) {
    intake = Intake;
    // Use addRequirements() here to declare subsystem dependencies. TODO
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    if (intake.doIHaveIt()) {
      intake.high_out();
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(.25)) {
      intake.low_in();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
   
    
  }
}
