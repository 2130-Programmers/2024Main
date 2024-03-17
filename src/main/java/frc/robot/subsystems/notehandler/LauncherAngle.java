// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notehandler;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherAngle extends SubsystemBase {
  /** Creates a new NoteHandler. */
  public LauncherAngle() {

  }

  /**
   * Move the launcher to specified angle: higher values move the intake higher, and thus the launcher points lower. 0 is horizontal, -8 is the ground
   * @param angle - the angle to move the launcher to, expressed in encoder ticks
   * @return true when the launcher has reached its target
   */
  public boolean moveToAngle(double angle) {
    if(angle > Constants.LauncherConstants.LAUNCHER_MAX_ANGLE) {
      throw new Error("Target launcher angle out of bounds");
    }

    double angleError = angle - (LauncherComponents.rotateLeft.getPosition().getValueAsDouble() + LauncherComponents.rotateRight.getPosition().getValueAsDouble())/2;

    if(angleError > 0) {
      LauncherComponents.rotateLeft.setControl(new VoltageOut(angleError * .75));
      LauncherComponents.rotateRight.setControl(new VoltageOut(angleError * .75));
    } else {
      LauncherComponents.rotateLeft.setControl(new VoltageOut(angleError * .15));
      LauncherComponents.rotateRight.setControl(new VoltageOut(angleError * .15));
    }

    return Math.abs(angleError) < .1;
  }

  /**
   * Move the launcher to the proper angle for shooting froma specific distance
   * @param distanceToTarget - the distance from the launcher to the speaker in meters
   */
  public void angleFromDistance(double distanceToTarget) {
    //Move to point in a line directly at the speaker(6ft off the ground)
    //Then add a small amount of extra angle that scales with distance from the target
    double targetAngle = (distanceToTarget * .7)-(Constants.LauncherConstants.RADIANS_TO_ENCODER * (Math.atan2(2, distanceToTarget)));
    if(targetAngle < Constants.LauncherConstants.LAUNCHER_MAX_ANGLE) {
      moveToAngle(targetAngle);
    }else{
      setRotatePower(0);
    }
  }

  /**
   * Move intake to bottom position and zero encoder
   * @return true when the intake has reached zero
   */
  public boolean zeroIntake() {
    if(true) {
      setRotatePower(-.4);
    }else{
      setRotatePower(0);
    }

    return !LauncherComponents.noteLimitSwitch.get();
  }

    /**
   * Set the power of rotation motors - positive moves the intake side up, and thus the launcher points lower.
   * Change to private ASAP
   * @param power - double, from -1 to 1
   */
  public void setRotatePower(double power) {
    if(Math.abs(power) > .05) {
      LauncherComponents.rotateLeft.set(power * .25);
      LauncherComponents.rotateRight.set(power * .25);
    } else {
      LauncherComponents.rotateRight.set(0);
      LauncherComponents.rotateLeft.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current arm angle", (LauncherComponents.rotateLeft.getPosition().getValueAsDouble() + LauncherComponents.rotateRight.getPosition().getValueAsDouble())/2);
  }
}