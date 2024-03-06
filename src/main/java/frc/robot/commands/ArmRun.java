// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmRun extends Command {
  Arm arm;
  double WristTarget;
  double Target;
  Timer timer;
  double m_secs;
  /** Move Arm to desired position
   *  @param Target is desired elbow position (in degrees from horizontal)
   *  @param WristTarget is desired wrist position (in degrees from normal to arm)
   *  @param m_secs is time (in seconds) to wait after starting motion
   */
  public ArmRun(Arm Arm, double Target, double WristTarget, double m_secs) {
    arm = Arm;
    this.Target = Target;
    this.WristTarget = WristTarget;
    timer = new Timer();  // MRG do you really need seperate Timers?
    this.m_secs = m_secs;
    addRequirements(arm); // here to declare subsystem dependencies.
  }
  
  /** Move Arm to desired position */
  public ArmRun(Arm Arm, double Target, double WristTarget) {
    this(Arm,Target,WristTarget,0.25);  // MRG: built in time due to wrist delay
  }

  /** set kP based on curent position and target, and start PID controllers */
  @Override
  public void initialize() {
    timer.start();
    //arm.pidCoefficient(Math.abs(Target - arm.getElbowPos()), Math.abs(WristTarget - arm.getWristPos()));
    arm.positionArm(Target);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(0.25)) arm.positionWrist(WristTarget);
    /*if (arm.getElbowPos() >= Target - ArmConstants.elbowTolerance
     && arm.getElbowPos() <= Target + ArmConstants.elbowTolerance) arm.stopElbow();
    if (arm.getWristPos() >= Target - ArmConstants.wristTolerance
     && arm.getWristPos() <= Target + ArmConstants.wristTolerance) arm.stopWrist();*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(m_secs);
  }
}
