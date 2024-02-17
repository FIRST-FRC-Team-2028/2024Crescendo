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
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    this.gyro = gyro;
    encoderLeft = climberLeft.getEncoder();
    encoderRight = climberRight.getEncoder();
    pidController = new PIDController( 0, 0, 0);
    climbController = climberLeft.getPIDController();
    climbController.setP(0);
  }

  public void extend(double speed) {
    climberRight.set(speed);
    climberLeft.set(speed);
  }

  public void levelme() {
    climberRight.set(pidController.calculate( gyro.getRoll().getValue(), 0.));
  }

  public void retract(double reatract) {
    climbController.setReference(reatract, ControlType.kPosition);
  }

  public void stop() {
    climberLeft.stopMotor();
    climberRight.stopMotor();
  }

  public double getPositionDriver() {
    return encoderLeft.getPosition();
  }

  public double getPositionLeveler() {
    return encoderRight.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
