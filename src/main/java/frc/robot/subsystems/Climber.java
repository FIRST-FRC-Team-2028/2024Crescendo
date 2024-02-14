// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
  CANSparkMax climberLeft =  new CANSparkMax(Constants.CANIDs.climb_left,MotorType.kBrushless);
  CANSparkMax climberRight =  new CANSparkMax(Constants.CANIDs.climb_right,MotorType.kBrushless);
  Pigeon2 gyro;
  RelativeEncoder encoder;

  /** Climber hooks would go up, hook, then go down. 
   *  The robot should maintain zero roll angle during climb.
  */
  public Climber(Pigeon2 gyro) {
    this.gyro = gyro;
    encoder = climberLeft.getEncoder();
  }

  public void extend(double speed) {
    climberRight.set(speed);
    climberLeft.set(speed);
  }

  public void stop() {
    climberLeft.stopMotor();
    climberRight.stopMotor();
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
