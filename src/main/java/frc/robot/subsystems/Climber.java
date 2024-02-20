// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class Climber extends SubsystemBase {
  CANSparkMax climberLeft =  new CANSparkMax(Constants.CANIDs.climb_left,MotorType.kBrushless);
  CANSparkMax climberRight =  new CANSparkMax(Constants.CANIDs.climb_right,MotorType.kBrushless);
  Pigeon2 gyro;
  RelativeEncoder encoderLeft;
  RelativeEncoder encoderRight;
  PIDController pidController;
  SparkPIDController climbController;

  /** Climber hooks would go up, hook, then go down. 
   *  The robot should maintain zero roll angle during climb.
  */
  public Climber(Pigeon2 gyro) {
    climberLeft.restoreFactoryDefaults();
    climberLeft.setInverted(true);
    climberLeft.setIdleMode(IdleMode.kBrake);
    climberRight.restoreFactoryDefaults();
    climberRight.setInverted(false);
    climberRight.setIdleMode(IdleMode.kBrake);
    this.gyro = gyro;
    encoderLeft = climberLeft.getEncoder();
    encoderRight = climberRight.getEncoder();
    encoderLeft.setPositionConversionFactor(Constants.ClimberConstants.encoderConversionFactor);
    encoderRight.setPositionConversionFactor(Constants.ClimberConstants.encoderConversionFactor);
    encoderLeft.setPosition(0.);   // MRG softlimits for safety
    encoderRight.setPosition(0.);
    climberLeft.setSoftLimit(SoftLimitDirection.kForward, Constants.ClimberConstants.extendLimit);
    climberRight.setSoftLimit(SoftLimitDirection.kForward, Constants.ClimberConstants.extendLimit);
    climberLeft.setSoftLimit(SoftLimitDirection.kReverse, Constants.ClimberConstants.retractLimit);  // probably zero
    climberRight.setSoftLimit(SoftLimitDirection.kReverse, Constants.ClimberConstants.retractLimit);
    climberLeft.enableSoftLimit(SoftLimitDirection.kForward,true);
    climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climberRight.enableSoftLimit(SoftLimitDirection.kForward,true);
    climberRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
    pidController = new PIDController( .15, 0, 0);
    climbController = climberLeft.getPIDController();
    climbController.setP(.1);
  }

  /** Extends the climber arms up
   * @param speed controls the speed of both motors
   */
  public void extend(double speed) {
    climberRight.set(speed);
    climberLeft.set(speed);
  }
  public void extend_right(double speed) {
    climberRight.set(speed);
  }
  public void extend_left(double speed) {
    climberLeft.set(speed);
  }
  public void getRoll() {
    System.out.println(gyro.getRoll());
  }
  
  
  
  /** levels the robot to 0 in the x-axis
   * 
   */
  public void levelme() {
    climberRight.set(pidController.calculate( gyro.getRoll().getValue(), 0.));
  }

  /** retracts to a position in closed loop
   * @param reatract position to retract to in inches from fully retracted
   */
  public void retract(double reatract) {

    climbController.setReference(reatract, ControlType.kPosition);
  }

  public void stop() {
    climberLeft.stopMotor();
    climberRight.stopMotor();
  }

  public void stop_left() {
    climberLeft.stopMotor();
  }

    public void stop_right() {
    climberRight.stopMotor();
  }

  public void zeroSoftLimit() {
    encoderLeft.setPosition(0.); 
    encoderRight.setPosition(0.);
  }

  public void enableSoftLimit(){
    climberLeft.enableSoftLimit(SoftLimitDirection.kForward,true);
    climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climberRight.enableSoftLimit(SoftLimitDirection.kForward,true);
    climberRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void disableLimit(){
    climberLeft.enableSoftLimit(SoftLimitDirection.kForward,false);
    climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
    climberRight.enableSoftLimit(SoftLimitDirection.kForward,false);
    climberRight.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }


  public double getPositionDriver() {
    return encoderLeft.getPosition();
  }

  public double getPositionLeveler() {
    return encoderRight.getPosition();
    //return 5;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
