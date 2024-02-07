// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/** Arm consists of:
 *     two tandem motors to drive the elbow,
 *     motor to position the intake
 *     absolute encoder to initialize the elbow position
 */
public class Arm extends SubsystemBase {
  private final CANSparkMax elbow;
  private final CANSparkMax elbow_follower;
  private final CANSparkMax wrist;

  private final RelativeEncoder elbow_encoder;
  private final RelativeEncoder wrist_encoder;
  private final SparkPIDController elbow_PidController;
  private final SparkPIDController wrist_PidController;
  private final AnalogInput boreHole;
  double Target;
  double WristTarget;
  double kp;
  double wkp;
  boolean IamDone;
  
  
  /** Creates a new Arm. */
  public Arm() {
    elbow = new CANSparkMax(Constants.CANIDs.elbow, MotorType.kBrushless);
    elbow_follower = new CANSparkMax(Constants.CANIDs.elbow_follower, MotorType.kBrushless);
    wrist = new CANSparkMax(Constants.CANIDs.wrist, MotorType.kBrushless);
    boreHole = new AnalogInput(Constants.ArmConstants.kAbsoluteEncoder);
    boreHole.setAverageBits(40);

    elbow.restoreFactoryDefaults();
    elbow_follower.restoreFactoryDefaults();
    wrist.restoreFactoryDefaults();
    //elbow_follower.follow(elbow, false);

    elbow.setIdleMode(IdleMode.kBrake);
    elbow_follower.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);

    elbow_encoder = elbow.getEncoder();
    wrist_encoder = wrist.getEncoder();

    elbow_PidController = elbow.getPIDController();
    wrist_PidController = wrist.getPIDController();

    elbow_PidController.setP(Constants.ArmConstants.kElbowP);
    elbow_PidController.setI(Constants.ArmConstants.kElbowI);
    elbow_PidController.setD(Constants.ArmConstants.kElbowD);

    wrist_PidController.setP(Constants.ArmConstants.kWristP);
    wrist_PidController.setI(Constants.ArmConstants.kWristI);
    wrist_PidController.setD(Constants.ArmConstants.kWristD);

    elbow.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kElbowForwardLimit); //elbow forward limit
    elbow.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kElbowReverseLimit); //elbow reverse limit
    wrist.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kWristForwardLimit); //wrist forward limit
    wrist.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kWristReverseLimit); //wrist reverse limit

    elbow.enableSoftLimit(SoftLimitDirection.kForward,IamDone);
    elbow.enableSoftLimit(SoftLimitDirection.kReverse, IamDone);

    //elbow_encoder.setPosition(abs2rel(boreHole.getAverageValue()));

  }

  /** Calibrate relative encoder from absolute encoder */
  double abs2rel(double absval){
    return Constants.ArmConstants.RelMin + Constants.ArmConstants.Ratio * (absval - Constants.ArmConstants.AbsMin);
  }
  public void elbowUp() {
   elbow.set(1);

SmartDashboard.putNumber("Encoder test", elbow_encoder.getPosition());
System.out.println("Insdie elbowup"); }
  public void elbowDown() {
    elbow.set(-1);
  }
  public void elbowDownSlow() {
    elbow.set(-.2);
  }
  public void elbowUpSlow() {
    elbow.set(.2);
  }
  
/* For test:
* stepping the motor 1 or 2 degrees in both directions
* */

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RelVal", elbow_encoder.getPosition());
    SmartDashboard.putNumber("AbsVal", boreHole.getAverageValue());


    System.out.println("TEST");
    // This method will be called once per scheduler run
  }

  /*public void retract() {
    wrist_PidController.setReference(Constants.ArmConstants.kRetractPosition, ControlType.kPosition);
    elbow_PidController.setReference(Constants.ArmConstants.kRetractPosition, ControlType.kPosition);
  }

  public void ground() {
    wrist_PidController.setReference(Constants.ArmConstants.kWristGround, ControlType.kPosition);
    elbow_PidController.setReference(Constants.ArmConstants.kElbowGround, ControlType.kPosition);
  }

  public void source() {
    wrist_PidController.setReference(Constants.ArmConstants.kWristSource, ControlType.kPosition);
    elbow_PidController.setReference(Constants.ArmConstants.kElbowSource, ControlType.kPosition);
  }

  public void amp() {
    wrist_PidController.setReference(Constants.ArmConstants.kWristAmp, ControlType.kPosition);
    elbow_PidController.setReference(Constants.ArmConstants.kElbowAmp, ControlType.kPosition);
  }
  
  public void speaker() {
    wrist_PidController.setReference(Constants.ArmConstants.kWristSpeaker, ControlType.kPosition);
    elbow_PidController.setReference(Constants.ArmConstants.kElbowSpeaker, ControlType.kPosition);
  }*/

  public void pidCoefficient(double distance, double wristDistance) {
    kp = 1 * Constants.ArmConstants.kElbowP / distance;
    wkp = 1 * Constants.ArmConstants.kWristP / wristDistance;
    elbow_PidController.setP(kp);
    wrist_PidController.setP(wkp);
  }

  public double getElbowPos() {
    return elbow_encoder.getPosition();
  }

  public double getWristPos() {
    return wrist_encoder.getPosition();
  }

  public void stopElbow() {
    elbow.stopMotor();
  }
  
  public void stopWrist() {
    wrist.stopMotor();
  }

  public void run(double targetW, double target) {
    wrist_PidController.setReference(WristTarget, ControlType.kPosition);
    elbow_PidController.setReference(Target, ControlType.kPosition);
  }

  public boolean amIDone() {
    return IamDone;
  }

  public void makeMeDone() {
    IamDone = true;
  }

  public void makeMeUndone() {
    IamDone = false;
  }
}
