// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Intake consists of:
 *     motor to drive intake rollers and belts, low_side
 *     motor to drive shooter rollers,
 *     sensor to determine when note has been acquired.
 */
public class Handler extends SubsystemBase {
  private final TalonSRX low_side;
  private final TalonSRX high_side;
  private final AnalogInput sensor;
  boolean doIHaveIt;

  //private final RelativeEncoder low_encoder;
  //private final RelativeEncoder high_encoder;

  //private final SparkPIDController low_PidController;
  //private final SparkPIDController high_PidController;
  /** Creates a new Intake. */
  public Handler() {
    low_side = new TalonSRX(Constants.CANIDs.low_side);
    high_side = new TalonSRX(Constants.CANIDs.high_side);

    low_side.configFactoryDefault();
    high_side.configFactoryDefault();

    low_side.setNeutralMode(NeutralMode.Brake); 
    high_side.setNeutralMode(NeutralMode.Coast);

   // low_encoder = low_side.getEncoder();
   // high_encoder = high_side.getEncoder();

    //low_PidController = low_side.getPIDController();
    //high_PidController = high_side.getPIDController();

    //low_PidController.setP(Constants.IntakeConstants.kLowP);
    //low_PidController.setI(Constants.IntakeConstants.kLowI);
    //low_PidController.setD(Constants.IntakeConstants.kLowD);
    
    //high_PidController.setP(Constants.IntakeConstants.kHighP);
    //high_PidController.setI(Constants.IntakeConstants.kHighI);
    //high_PidController.setD(Constants.IntakeConstants.kHighD);
    sensor = new AnalogInput(Constants.IntakeConstants.SENSORPORT);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public boolean doIHaveIt() {
    return doIHaveIt;
  }
 
  /**Returns true if we sense one */
  public boolean sense() {
    return false;
  }

  public void iHaveIt() {
    doIHaveIt = true;
  }

  public void iDontHaveIt() {
    doIHaveIt = false;
  }



  public void high_out() {
    high_side.set(TalonSRXControlMode.PercentOutput, Constants.IntakeConstants.kHighOutSpeed);
  }

  public void low_in() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.IntakeConstants.kLowInSpeed);
  }

  public void low_out() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.IntakeConstants.kLowOutSpeed);
  }

  public void stop() {
    high_side.set(TalonSRXControlMode.PercentOutput, 0);
    low_side.set(TalonSRXControlMode.PercentOutput, 0);
  }
}
 