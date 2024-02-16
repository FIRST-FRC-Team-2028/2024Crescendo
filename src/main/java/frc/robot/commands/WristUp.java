// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class WristUp extends Command {
  Arm subsys;
  double speed;
  /** Creates a new WristUp. */
  public WristUp(Arm subsys, double speed) {
    this.subsys = subsys;
    this.speed = speed;
     if (Constants.ARM_AVAILABLE){
    addRequirements(subsys); //here to declare subsystem dependencies.
      }
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsys.moveWrist(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsys.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
