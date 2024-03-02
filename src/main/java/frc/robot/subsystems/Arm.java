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
import edu.wpi.first.wpilibj2.command.Command;
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
  private final AnalogInput elbowAbs;
  private final AnalogInput wristAbs;
  
  double latestTarget;
  double latestTargetW;
  double kp;
  double wkp;
  boolean IamDone;
  boolean armSafety = true;   // true for arm motion enabled
  boolean armSafetyw = true;   // true for wrist motion enabled
  
  /** The Arm:
   *    o moves the handler (relative to the robot)
   *    o actuates at the base (called elbow)
   *       and the tip (called the wrist)
   *    o uses absolute encoders to calibrate relative encoders for each actuator
   */
  public Arm() {
    elbow = new CANSparkMax(Constants.CANIDs.elbow, MotorType.kBrushless);
    elbow_follower = new CANSparkMax(Constants.CANIDs.elbow_follower, MotorType.kBrushless);
    wrist = new CANSparkMax(Constants.CANIDs.wrist, MotorType.kBrushless);
    elbowAbs = new AnalogInput(Constants.ArmConstants.kAbsoluteEncoder);
    wristAbs = new AnalogInput(Constants.ArmConstants.kAbsoluteEncoderW);
    elbowAbs.setAverageBits(40);
    wristAbs.setAverageBits(40);
    //boreHoleW = new AnalogInput(Constants.ArmConstants.kAbsoluteEncoderW);
    //boreHoleW.setAverageBits(40);

    elbow.restoreFactoryDefaults();
    elbow_follower.restoreFactoryDefaults();
    wrist.restoreFactoryDefaults();
    elbow_follower.follow(elbow, true);

    elbow.setIdleMode(IdleMode.kBrake);
    elbow_follower.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);
    wrist.setInverted(true);

    elbow_encoder = elbow.getEncoder();
    wrist_encoder = wrist.getEncoder();

    elbow_encoder.setPositionConversionFactor(Constants.ArmConstants.elbowEncoderFactor);
    wrist_encoder.setPositionConversionFactor(Constants.ArmConstants.wristEncoderFactor);
    elbow_encoder.setPosition(abs2rel(elbowAbs.getAverageValue()));
    wrist_encoder.setPosition(abs2relw(wristAbs.getAverageValue()));

    
    elbow.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kElbowForwardLimit); //elbow forward limit
    elbow.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kElbowReverseLimit); //elbow reverse limit
    wrist.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kWristForwardLimit); //wrist forward limit
    wrist.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kWristReverseLimit); //wrist reverse limit

    elbow.enableSoftLimit(SoftLimitDirection.kForward,true);
    elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);
    wrist.enableSoftLimit(SoftLimitDirection.kForward,true);
    wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);

    elbow_PidController = elbow.getPIDController();
    wrist_PidController = wrist.getPIDController();

    elbow_PidController.setP(Constants.ArmConstants.kElbowP);
    elbow_PidController.setI(Constants.ArmConstants.kElbowI);
    elbow_PidController.setD(Constants.ArmConstants.kElbowD);

    wrist_PidController.setP(Constants.ArmConstants.kWristP);
    wrist_PidController.setI(Constants.ArmConstants.kWristI);
    wrist_PidController.setD(Constants.ArmConstants.kWristD);

    elbow.setOpenLoopRampRate(Constants.ArmConstants.kElbowRampRate);
    elbow.setClosedLoopRampRate(Constants.ArmConstants.kElbowRampRate);
    wrist.setOpenLoopRampRate(Constants.ArmConstants.kWristRampRate);
    wrist.setClosedLoopRampRate(Constants.ArmConstants.kWristRampRate);
  }

  /** Calibrate relative encoder from absolute encoder for the elbow*/
  double abs2rel(double absval){
    System.out.println("calibrate Arm: Rmin + Ratio*(aval-amin) = rval");
    double val = Constants.ArmConstants.RelMin + Constants.ArmConstants.Ratio * (absval - Constants.ArmConstants.AbsMin);
    System.out.println("               "+Constants.ArmConstants.RelMin+" + " + Constants.ArmConstants.Ratio+ " * ("+absval+" - "+Constants.ArmConstants.AbsMin+") = "+val);
    return Constants.ArmConstants.RelMin + Constants.ArmConstants.Ratio * (absval - Constants.ArmConstants.AbsMin);
  }
  /** Calibrate relative encoder from absolute encoder for the wrist*/
  double abs2relw(double absval){
    System.out.println("calibrate Wrist");
    double val = Constants.ArmConstants.RelMinW + Constants.ArmConstants.RatioW * (absval - Constants.ArmConstants.AbsMinW);
    System.out.println("               "+Constants.ArmConstants.RelMinW+" + " + Constants.ArmConstants.RatioW+ " * ("+absval+" - "+Constants.ArmConstants.AbsMinW+") = "+val);
    return Constants.ArmConstants.RelMinW + Constants.ArmConstants.RatioW * (absval - Constants.ArmConstants.AbsMinW);
  }

  /** Sets the arm motors to coast */
  public void setCoastMode() {
    elbow.setIdleMode(IdleMode.kCoast);
    elbow_follower.setIdleMode(IdleMode.kCoast);
    wrist.setIdleMode(IdleMode.kCoast);
  }
  
  /** Sets the arm motors to brake */
  public void setBrakeMode() {
    elbow.setIdleMode(IdleMode.kBrake);
    elbow_follower.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);
  }

 
  /** Elbow up open loop control higher speed */
  public void elbowUp() {
   if(armSafety)elbow.set(.7);

    //SmartDashboard.putNumber("Encoder test", elbow_encoder.getPosition());
    //System.out.println("Insdie elbowup"); 
  }

  /** Elbow down open loop control higher speed */
  public void elbowDown() {
    if(armSafety)elbow.set(-.7);
  }

  /** Elbow down open loop control lower speed */
  public void elbowDownSlow() {
    if(armSafety)elbow.set(-.2);
  }
  
  /** Elbow up open loop control  lower speed*/
  public void elbowUpSlow() {
    if(armSafety)elbow.set(.2);
  }

  /** move arm - open loop
   * 
   * @param speed  positive is up from ground
  */
  public void moveArm(double speed){
    if(armSafety)elbow.set(speed);
  }

  public double getElbowCurrent() {
    return elbow.getOutputCurrent();
  }

  public double getElbowCurrentw() {
    return wrist.getOutputCurrent();
  }
  

  /** moveWrist 
   * @param speed is positive in the same axis as the arm
  */
  public void moveWrist(double speed) {
    if(armSafetyw) wrist.set(speed);
  }

  double[] currentHist = {0.,0.,0.,0.,0.};
  int currP = 0;
  double avgCurrent = 0.;
  double[] currentHistw = {0.,0.,0.,0.,0.};
  int currPw = 0;
  double avgCurrentw = 0.;

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("AbsWrist", wristAbs.getAverageValue());
    double currentCurrent = getElbowCurrent();
    //SmartDashboard.putNumber("Current", currentCurrent);
    avgCurrent += currentCurrent/5. - currentHist[currP]/5.;
    currentHist[currP] = currentCurrent;
    currP = (currP+1)%5;
    SmartDashboard.putNumber("ElbowRelVal", elbow_encoder.getPosition());
    SmartDashboard.putNumber("ElbowAbsVal", elbowAbs.getAverageValue());
    SmartDashboard.putNumber("WristRelVal", wrist_encoder.getPosition());
    SmartDashboard.putNumber("WristAbsVal", wristAbs.getAverageValue());
    if(avgCurrent>Constants.ArmConstants.ElbowCurrentLimit) {
      //for (double each: currentHist) System.out.print(" "+each);
      //System.out.println(" => avg: "+avgCurrent);
      armSafety = false;
    }
    //SmartDashboard.putBoolean("Elbow Warning", armSafety);


    double currentCurrentw = getElbowCurrentw();
    //SmartDashboard.putNumber("WCurrent", currentCurrentw);
    avgCurrentw += currentCurrentw/5. - currentHistw[currPw]/5.;
    currentHistw[currPw] = currentCurrentw;
    currPw = (currPw+1)%5;
    //SmartDashboard.putNumber("WRelVal", wrist_encoder.getPosition());
    //SmartDashboard.putNumber("AbsVal", boreHolew.getAverageValue());
    if(avgCurrentw>Constants.ArmConstants.ElbowCurrentLimit) armSafetyw = false;
    //SmartDashboard.putBoolean("Wrist Warning", armSafetyw);

    //System.out.println("TEST");
    // This method will be called once per scheduler run
  }



  /** Allow the arm to move  (after safety shutdown)*/
  public void rearmArm() {
    armSafety = true;
    armSafetyw = true;
    elbow_encoder.setPosition(abs2rel(elbowAbs.getAverageValue()));
    wrist_encoder.setPosition(abs2relw(wristAbs.getAverageValue()));
  }

  /** Disable motion of the arm */
  public void disarmArm() {
    armSafety = false;
    armSafetyw = false;
  }

  /** closed loop control arm to target
   * @param target angle from floor, degrees
   */
  public void positionArm(double target) {
    elbow_PidController.setReference(target, CANSparkMax.ControlType.kPosition);
    latestTarget = target;
  }

  /** closed loop control wrist to target
   * @param target angle trom perpendicular, degrees positive in same axis as elbow
   */
  public void positionWrist(double target) {
    wrist_PidController.setReference(target, CANSparkMax.ControlType.kPosition);
    latestTargetW = target;
  }

  /** Adjust elbow PID target 
   * @see positionArm
  */
  public void retargetElbow(double Adjustment) {
     latestTarget += Adjustment;
    elbow_PidController.setReference(latestTarget, CANSparkMax.ControlType.kPosition);
  }

  /** Adjust wrist PID target 
   * @see positionWrist
  */
  public void retargetWrist(double Adjustment) {
    latestTargetW += Adjustment;
    wrist_PidController.setReference(latestTargetW, CANSparkMax.ControlType.kPosition);
  }
  


  public void pidCoefficient(double distance, double wristDistance) {
    kp = 1 * Constants.ArmConstants.kElbowP / distance;
    wkp = 1 * Constants.ArmConstants.kWristP / wristDistance;
    elbow_PidController.setP(kp);
    wrist_PidController.setP(wkp);
  }

  /** Get Elbow position
   *   degrees up from parallel to floor
   */
  public double getElbowPos() {
    return elbow_encoder.getPosition();
  }

  /** Get wrist position
   *   degrees from perpendicular to arm,
   *   rotation direction same as arm
   */
  public double getWristPos() {
    return wrist_encoder.getPosition();
  }

  public void stopElbow() {
    elbow.stopMotor();
  }
  
  public void stopWrist() {
    wrist.stopMotor();
  }

  public void stopIt() {
    wrist.stopMotor();
    elbow.stopMotor();
  }

  /** set closed loop position of both elbow and wrist 
   * @see positionArm, positionWrist
  */
  public void run(double targetW, double target) {
    positionArm(target);
    positionWrist(targetW);
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
