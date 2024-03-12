// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber climber;
  PIDController pitchController;
  Pigeon2 gyro;
  Arm arm;
  /** Climb:
   * retract the climbers to the retracted position 
   * while maintaining levelness
   * In both axies
   * roll angle tells us when to start the leveler arm to level the robot
   * pitch angle tells us when to tweak the arm
   */
  public Climb(Climber climber, Pigeon2 gyro, Arm arm) {
    this.climber = climber;
    this.gyro = gyro;
    this.arm = arm;
    addRequirements(climber);  // here to declare subsystem dependencies.
    // use a PIDController to control the balance arm based on gyro data
    pitchController = new PIDController(0, 0, 0);
  }

  // Called when the command is initially scheduled.
  // start retracting the arms
  @Override
  public void initialize() {
    climber.retract(Constants.ClimberConstants.RetractPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Maintain level, ie run the PIDController
  @Override
  public void execute() {
    climber.levelme();
    System.out.println(climber.getPositionDriver());
    System.out.println(climber.getPositionLeveler());
    if (Constants.ARM_AVAILABLE)
    arm.retargetElbow(-pitchController.calculate(gyro.getPitch().getValue(), 0));
  }

  // Called once the command ends or is interrupted.
  // Don't let the robot fall after it has climbed
  @Override
  public void end(boolean interrupted) {
    climber.retract(climber.getPositionDriver());

  }
  
  double small = .5;  //inches
  // Returns true when the command should end.
  // When either of the arms reaches the retracted position
  @Override
  public boolean isFinished() {
    return climber.getPositionLeveler() <= Constants.ClimberConstants.RetractPosition + small
    || climber.getPositionDriver() <= Constants.ClimberConstants.RetractPosition + small;
  }
}
