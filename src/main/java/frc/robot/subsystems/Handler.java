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
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

/** Handler consists of:
 *     motor to drive handler rollers and belts, low_side
 *     motor to drive shooter rollers, (high_side)
 *     sensor to determine when note has been acquired.
 */
public class Handler extends SubsystemBase {
  private final TalonSRX low_side;
  private final TalonSRX high_side;
  private final ColorSensorV3 sensor;
  boolean doIHaveIt;

  //private final RelativeEncoder low_encoder;
  //private final RelativeEncoder high_encoder;

  //private final SparkPIDController low_PidController;
  //private final SparkPIDController high_PidController;
  /** Creates a new Handlerhandler. */
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

    //low_PidController.setP(Constants.handlerConstants.kLowP);
    //low_PidController.setI(Constants.handlerConstants.kLowI);
    //low_PidController.setD(Constants.handlerConstants.kLowD);
    
    //high_PidController.setP(Constants.handlerConstants.kHighP);
    //high_PidController.setI(Constants.handlerConstants.kHighI);
    //high_PidController.setD(Constants.handlerConstants.kHighD);
    sensor = new ColorSensorV3(Constants.ColorConstants.sensorPort);
  }
  
  static final double closeHue = 0.05;  // How close to the Hue data [0-1.] is good enough

  /** return true if sensor sees a note */
  public boolean useSensor() {
    // return sensor.get();  TODO
    Color notesensor = sensor.getColor();
    SmartDashboard.putString("Sensor", notesensor.toHexString());
    float[] george={0.f,0.f,0.f};
    george = java.awt.Color.RGBtoHSB((int)(notesensor.red  *255), 
                                            (int)(notesensor.green*255), 
                                            (int)(notesensor.blue *255), george);
                                            double diffHue = Math.abs(george[0] - Constants.ColorConstants.NoteHue);
    if ( diffHue < closeHue)return true;
                                        
    return false;
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
    high_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kHighOutSpeed);
  }

  public void low_in() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kLowInSpeed);
  }

  public void low_out() {
    low_side.set(TalonSRXControlMode.PercentOutput, Constants.HandlerConstants.kLowOutSpeed);
  }

  public void stop() {
    high_side.set(TalonSRXControlMode.PercentOutput, 0);
    low_side.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**ShootIt command
   * runs the low side to get the note to the high_side rollers
   * runs the high_side rollers to shoot
   * turns off all 
   */
  public Command shootIt() {
    return runOnce( () -> { high_out(); })
          .andThen( () -> { low_in(); })
          .andThen(new WaitCommand(.5))
          .andThen( () -> { stop(); })
      ;
  }
}
 