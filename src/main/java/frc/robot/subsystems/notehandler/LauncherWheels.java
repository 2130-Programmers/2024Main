// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notehandler;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherWheels extends SubsystemBase {
  /** Creates a new LauncherWheels. */
  public LauncherWheels() {}

    /**
   * Check if the launcher is at or above the specified speed
   * @param rpm - the minimum speed
   * @return true if the launcher is at or above the minimum speed
   */
  public boolean launcherAtSpeed(double rpm) {
    return (LauncherComponents.launcherBottom.getRotorVelocity().getValueAsDouble() * 60 >= rpm) && (LauncherComponents.launcherTop.getRotorVelocity().getValueAsDouble() * 60 >= rpm);
  }


  private double launcherSpeed() {
    return (LauncherComponents.launcherBottom.getRotorVelocity().getValueAsDouble() * 30 + LauncherComponents.launcherTop.getRotorVelocity().getValueAsDouble() * 30);
  }

  /**
   * Set the power of launcher motors - positive moves the note in
   * @param power - double, from -1 to 1
   */
  public void setLaunchPower(double power) {
    LauncherComponents.launcherTop.set(power);
    LauncherComponents.launcherBottom.set(power);
  }

  /**
   * Set the speed of the launcher in RPM
   * @param rpm - the value to set to both launch wheels
   */
  public void setLaunchRpm(double rpm) {
    LauncherComponents.launcherBottom.setControl(new VelocityVoltage(rpm/60));
    LauncherComponents.launcherTop.setControl(new VelocityVoltage(rpm/60));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Launcher At Speed", launcherAtSpeed(5800));
    SmartDashboard.putNumber("Launcher Speed", launcherSpeed());
  }
}
