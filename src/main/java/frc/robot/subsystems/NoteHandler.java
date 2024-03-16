// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class NoteHandler extends SubsystemBase {
  private static final DigitalInput
  bottomDetectionProx = new DigitalInput(0),
  noteLimitSwitch = new DigitalInput(1);

  private static final TalonFX
  launcherTop = new TalonFX(33),
  launcherBottom = new TalonFX(32),
  rotateLeft = new TalonFX(30),
  rotateRight = new TalonFX(31);

  private static final CANSparkMax
  intakeTop = new CANSparkMax(35, MotorType.kBrushless),
  intakeBottom = new CANSparkMax(34, MotorType.kBrushless);

  /** Creates a new NoteHandler. */
  public NoteHandler() {
    rotateRight.setInverted(true);
    rotateLeft.setInverted(false);
    rotateLeft.setNeutralMode(NeutralModeValue.Brake);
    rotateRight.setNeutralMode(NeutralModeValue.Brake);
    launcherTop.setInverted(false);
    launcherBottom.setInverted(true);
    intakeTop.setInverted(false);
    intakeBottom.setInverted(false);

    rotateLeft.setPosition(0);
    rotateRight.setPosition(0);
  }

  /**
   * Move the launcher to specified angle: higher values move the intake higher, and thus the launcher points lower. 0 is the resting position and used for picking up nots off the ground
   * @param angle - the angle to move the launcher to, expressed in encoder ticks
   * @return true when the launcher has reached its target
   */
  public void moveToAngle(double angle) {
    if(angle > Constants.LauncherConstants.LAUNCHER_MAX_ANGLE) {
      throw new Error("Target launcher angle out of bounds");
    }

    double angleError = angle - (rotateLeft.getPosition().getValueAsDouble() + rotateRight.getPosition().getValueAsDouble())/2;

    if (angle < 12){
      setRotatePower(.3);
    } else {
      setRotatePower(.15);
    }

  }

  /**
   * Move the launcher to the proper angle for shooting froma specific distance
   * @param distanceToTarget - the distance from the launcher to the speaker in meters
   */
  public void angleFromDistance(double distanceToTarget) {
    //Move to point in a line directly at the speaker(6ft off the ground)
    //Then add a small amount of extra angle that scales with distance from the target
    double targetAngle = Constants.LauncherConstants.RADIANS_TO_ENCODER * (Math.atan2(2, distanceToTarget));
    if(targetAngle < 15) {
      moveToAngle(targetAngle);
    }else{
      moveToAngle(0);
    }
  }

  /**
   * Move intake to bottom position and zero encoder
   * @return true when the intake has reached zero
   */
  public boolean zeroIntake() {
    if(bottomDetectionProx.get()) {
      setRotatePower(-.4);
    }else{
      setRotatePower(0);
    }

    return !bottomDetectionProx.get();
  }


  /**
   * Check if there is a note present
   * @return - true if there is a note in the launcher
   */
  public boolean notePresent() {
    return noteLimitSwitch.get();
  }

  boolean readyLauncher(int targetSpeed) {
    // launcherTop.setControl()
    return false;
  }

  /**
   * Run intake until note reaches photoeye
   * @return true when the note is in position
   */
  public boolean intakeNote() {
    if(!noteLimitSwitch.get()) {
      setIntakePower(.2);
    }else{
      setIntakePower(0);
    }

    return noteLimitSwitch.get();
  }

  /**
   * Set the power of intake motors - positive moves the note in
   * @param power - double, from -1 to 1
   */
  public void setIntakePower(double power) {
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
    if(Math.abs(power) > .05) {
      rotateLeft.set(power * .25);
      rotateRight.set(power * .25);
    } else {
      rotateRight.set(0);
      rotateLeft.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Launcher angle measurement", (rotateLeft.getPosition().getValueAsDouble() + rotateRight.getPosition().getValueAsDouble())/2);
  }
}