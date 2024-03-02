// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteHandler extends SubsystemBase {
  private static final DigitalInput
  bottomDetectionProx = new DigitalInput(0),
  noteLaserEye = new DigitalInput(0);

  private static final TalonFX
  launcherTop = new TalonFX(0),
  launcherBottom = new TalonFX(0),
  rotateLeft = new TalonFX(0),
  rotateRight = new TalonFX(0);

  private static final CANSparkMax
  intakeTop = new CANSparkMax(0, MotorType.kBrushless),
  intakeBottom = new CANSparkMax(0, MotorType.kBrushless);

  /** Creates a new NoteHandler. */
  public NoteHandler() {
  }

  /**
   * Move the launcher to specified angle: higher values move the intake higher, and thus the launcher points lower. 0 is the resting position and used for picking up nots off the ground
   * @param angle - the angle to move the launcher to, expressed in encoder ticks
   * @return true when the launcher has reached its target
   */
  public boolean moveToAngle(double angle) {
    if(angle > Constants.LauncherConstants.LAUNCHER_MAX_ANGLE) {
      throw new Error("Target launcher angle out of bounds");
    }

    double angleError = (rotateLeft.getPosition().getValueAsDouble() + rotateRight.getPosition().getValueAsDouble())/2;

    if(Math.abs(angleError) > Constants.LauncherConstants.ANGLE_DEADZONE) {
      setRotatePower(angleError * Constants.LauncherConstants.ANGLE_POWER);
      return true;
    }else{
      setRotatePower(0);
      return false;
    }
  }

  /**
   * Move intake to bottom position and zero encoder
   * @return true when the intake has reached zero
   */
  public boolean zeroIntake() {
    if(bottomDetectionProx.get()) {
      setRotatePower(.25);
    }else{
      setRotatePower(0);
    }

    return !bottomDetectionProx.get();
  }

  /**
   * Run intake until note reaches photoeye
   * @return true when the note is in position
   */
  public boolean intakeNote() {
    if(!noteLaserEye.get()) {
      setIntakePower(.5);
    }else{
      setIntakePower(0);
    }

    return noteLaserEye.get();
  }

  /**
   * Set the power of intake motors - positive moves the note in
   * @param power - double, from -1 to 1
   */
  private void setIntakePower(double power) {
    intakeTop.set(power);
    intakeBottom.set(power);
  }

    /**
   * Set the power of launcher motors - positive moves the note in
   * @param power - double, from -1 to 1
   */
  public void setLaunchPower(double power) {
    launcherTop.set(power);
    launcherBottom.set(power);
  }

    /**
   * Set the power of rotation motors - positive moves the intake side up, and thus the launcher points lower.
   * Change to private ASAP
   * @param power - double, from -1 to 1
   */
  public void setRotatePower(double power) {
    rotateLeft.set(power);
    rotateRight.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
