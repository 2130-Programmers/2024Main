// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorControl extends SubsystemBase {
  private final CANSparkMax motor1 = new CANSparkMax(10, MotorType.kBrushed), motor2 = new CANSparkMax(11, MotorType.kBrushed);
  private final VictorSP motor3 = new VictorSP(3), motor4 = new VictorSP(5);
  
  /** Creates a new motorControl. */
  public MotorControl() {
    SmartDashboard.putNumber("Powerlevel", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = SmartDashboard.getNumber(getName(), 0);

    motor1.set(power * -1);
    motor2.set(power * -1);
    motor3.set(power * 1);
    motor4.set(power * 1);
  }
}
