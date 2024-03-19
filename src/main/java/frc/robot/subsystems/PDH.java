// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDH extends SubsystemBase {
  private PowerDistribution revPDH = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new PDH. */
  public PDH() {}

  /**
   * Turn on or off the note status leds
   * @param state - true to turn the lights on
   */
  public void setNoteLeds(boolean state) {
    revPDH.setSwitchableChannel(state);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PDH Current", revPDH.getTotalCurrent());
  }
}
