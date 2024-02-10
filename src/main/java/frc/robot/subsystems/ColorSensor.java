// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;                        // Maddox: Color sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);   
  /** Creates a new ColorSensor. */
  public ColorSensor() {
    
  }

  @Override
  public void periodic() {
            Color detectedColor = m_colorSensor.getColor();

        double IR = m_colorSensor.getIR();

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

        int proximity = m_colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);

        CommandScheduler.getInstance().run();
    // This method will be called once per scheduler run
  }
}
