// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notehandler;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LauncherIntake extends SubsystemBase {
  /** Creates a new LauncherIntake. */
  public LauncherIntake() {}
  
  /**
   * Check if there is a note present
   * @return - true if there is a note in the launcher
   */
  public boolean notePresent() {
    return LauncherComponents.noteLimitSwitch.get();
  }

  /**
   * Set the power of intake motors - positive moves the note in
   * @param power - double, from -1 to 1
   */
  public void setIntakePower(double power) {
    LauncherComponents.intakeTop.set(power);
    LauncherComponents.intakeBottom.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note switches", notePresent());
    RobotContainer.pdh.setNoteLeds(notePresent());
  }
}
