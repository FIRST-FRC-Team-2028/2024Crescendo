// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmRun extends Command {
  Arm arm;
  double WristTarget;
  double Target;
  Timer external_timer;
  Timer internal_timer;
  double m_secs;
  /** Move Arm to desired position */
  public ArmRun(Arm Arm, double Target, double WristTarget, double m_secs) {
    arm = Arm;
    this.Target = Target;
    this.WristTarget = WristTarget;
    external_timer = new Timer();
    internal_timer = new Timer();
    this.m_secs = m_secs;
    addRequirements(arm); // here to declare subsystem dependencies.
  }

  /** set kP based on curent position and target, and start PID controllers */
  @Override
  public void initialize() {
    external_timer.start();
    internal_timer.start();
    //arm.pidCoefficient(Math.abs(Target - arm.getElbowPos()), Math.abs(WristTarget - arm.getWristPos()));
    arm.positionArm(Target);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (internal_timer.hasElapsed(0.25)) arm.positionWrist(WristTarget);
    /*if (arm.getElbowPos() >= Target - ArmConstants.elbowTolerance
     && arm.getElbowPos() <= Target + ArmConstants.elbowTolerance) arm.stopElbow();
    if (arm.getWristPos() >= Target - ArmConstants.wristTolerance
     && arm.getWristPos() <= Target + ArmConstants.wristTolerance) arm.stopWrist();*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    external_timer.stop();
    external_timer.reset();
    internal_timer.stop();
    internal_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return external_timer.hasElapsed(m_secs);
  }
}
