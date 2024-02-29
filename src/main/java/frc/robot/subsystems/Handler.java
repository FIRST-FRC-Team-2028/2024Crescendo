// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.HandlerConstants;

/** Handler consists of:
 *     motor to drive handler rollers and belts, low_side
 *     motor to drive shooter rollers, (high_side)
 *     sensor to determine when note has been acquired.
 */
public class Handler extends SubsystemBase {
  private final TalonSRX low_side;
  private final TalonSRX high_side_follower;
  private final TalonSRX high_side;
  //private final ColorSensorV3 sensor;
  private final DigitalInput sensord;
  boolean doIHaveIt;

  //private final RelativeEncoder low_encoder;
  //private final RelativeEncoder high_encoder;

  //private final SparkPIDController low_PidController;
  //private final SparkPIDController high_PidController;
  /** Creates a new Handlerhandler. */
  public Handler() {
    low_side = new TalonSRX(Constants.CANIDs.low_side);
    high_side = new TalonSRX(Constants.CANIDs.high_side);
    high_side_follower = new TalonSRX(Constants.CANIDs.high_side_follower);

    low_side.configFactoryDefault();
    high_side.configFactoryDefault();
    high_side_follower.configFactoryDefault();

    //high_side_follower.setInverted(true);
    high_side_follower.follow(high_side);

    low_side.setNeutralMode(NeutralMode.Coast); 
    high_side.setNeutralMode(NeutralMode.Coast);

   // low_encoder = low_side.getEncoder();
   // high_encoder = high_side.getEncoder();

    //low_PidController = low_side.getPIDController();
    //high_PidController = high_side.getPIDController();

    //low_PidController.setP(Constants.handlerConstants.kLowP);
    //low_PidController.setI(Constants.handlerConstants.kLowI);
    //low_PidController.setD(Constants.handlerConstants.kLowD);
    
    //high_PidController.setP(Constants.IntakeConstants.kHighP);
    //high_PidController.setI(Constants.IntakeConstants.kHighI);
    //high_PidController.setD(Constants.IntakeConstants.kHighD);

      //sensor = new ColorSensorV3(Constants.ColorConstants.sensorPort);
      sensord = new DigitalInput(Constants.ColorConstants.sensordPort);
    
  }
  
  static final double closeHue = 0.05;  // How close to the Hue data [0-1.] is good enough

  /** return true if sensor sees a note */
  public boolean useSensor() {
    return sensord.get();
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

  /** set high_side motors to high speed out */
  public void high_out() {
    high_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kHighOutSpeed);
  }

  /*public void printVoltage() {
    System.out.println(low_side.getBusVoltage());
    System.out.println(low_side.getStatorCurrent());
  }*/

  /** set low_side motor to pickup speed */
  public void low_PickUp() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kLowPickUpSpeed);
  }

  /** set low_side motor to push note into high_side */
  public void low_ToHigh() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kLowToHighSpeed);
  }

  /** set low_side motor low-speed out */
  public void low_out() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kLowOutSpeed);
  }

  /** set high_side motor to amp-speed shoot */
  public void shootInAmpHigh() {
    high_side.set(TalonSRXControlMode.PercentOutput, HandlerConstants.kHighAmpSpeed);
  }

  /** set low_side motor to out amp_speed */
  public void shootInAmpLow() {
    low_side.set(TalonSRXControlMode.PercentOutput, HandlerConstants.kLowAmpSpeed);
  }




  /** stop both low_side and high_side motors */
  public void stop() {
    high_side.set(TalonSRXControlMode.PercentOutput, 0);
    low_side.set(TalonSRXControlMode.PercentOutput, 0);
    // System.out.println("Works LowStop");
  }

  /**ShootIt command
   * runs the low side to get the note to the high_side rollers
   * runs the high_side rollers to shoot
   * turns off all 
   */
  public Command shootIt() {
    return runOnce( () -> { high_out(); })
          .andThen(new WaitCommand(.5))
          .andThen( () -> { low_ToHigh(); })
          .andThen(new WaitCommand(.5))
          .andThen( () -> { stop(); })
      ;
  }
}
 