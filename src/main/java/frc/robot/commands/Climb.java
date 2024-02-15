// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber climber;
  /** Climb:
   * retract the climbers to the retracted position 
   * while maintaining levelness
   */
  public Climb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);  // here to declare subsystem dependencies.
    // use a PIDController to control the balance arm based on gyro data
  }

  // Called when the command is initially scheduled.
  // start retracting the arms
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Maintain level, ie run the PIDController
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  // Don't let the robot fall after it has climbed
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // When either of the arms reaches the retracted position
  @Override
  public boolean isFinished() {
    return false;
  }
}
