// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteHandler extends SubsystemBase {
  //Set up inputs for safety prox and note detection switch
  private static final DigitalInput
  bottomDetectionProx = new DigitalInput(0),
  noteLimitSwitch = new DigitalInput(1);

  //Define rotation and launch motors
  private static final TalonFX
  launcherTop = new TalonFX(33),
  launcherBottom = new TalonFX(32),
  rotateLeft = new TalonFX(30),
  rotateRight = new TalonFX(31);

  //Define intake motors
  private static final CANSparkMax
  intakeTop = new CANSparkMax(35, MotorType.kBrushless),
  intakeBottom = new CANSparkMax(34, MotorType.kBrushless);

  /** Creates a new NoteHandler. */
  public NoteHandler() {
    //Set the inversion for each motor so they work together
    rotateRight.setInverted(true);
    rotateLeft.setInverted(false);
    launcherTop.setInverted(false);
    launcherBottom.setInverted(true);
    intakeTop.setInverted(false);
    intakeBottom.setInverted(false);

    //Set intake and angle motors to brake and launcher to coast
    rotateLeft.setNeutralMode(NeutralModeValue.Brake);
    rotateRight.setNeutralMode(NeutralModeValue.Brake);
    intakeTop.setIdleMode(IdleMode.kBrake);
    intakeBottom.setIdleMode(IdleMode.kBrake);
    launcherTop.setNeutralMode(NeutralModeValue.Coast);
    launcherBottom.setNeutralMode(NeutralModeValue.Coast);

    //Zero the launcher when code starts
    rotateLeft.setPosition(-8);
    rotateRight.setPosition(-8);
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

    double angleError = angle - (rotateLeft.getPosition().getValueAsDouble() + rotateRight.getPosition().getValueAsDouble())/2;

    // rotateLeft.

    return angleError < .1;
  }

  /**
   * Check if the launcher is at or above the specified speed
   * @param rpm - the minimum speed
   * @return true if the launcher is at or above the minimum speed
   */
  public boolean launcherAtSpeed(double rpm) {
    return (launcherBottom.getRotorVelocity().getValueAsDouble() * 60 >= rpm) && (launcherTop.getRotorVelocity().getValueAsDouble() * 60 >= rpm);
  }

  /**
   * Set the speed of the launcher in RPM
   * @param rpm - the value to set to both launch wheels
   */
  public void setLaunchRpm(double rpm) {
    launcherBottom.setControl(new VelocityVoltage(rpm/60));
    launcherTop.setControl(new VelocityVoltage(rpm/60));
  }

  /**
   * Move the launcher to the proper angle for shooting froma specific distance
   * @param distanceToTarget - the distance from the launcher to the speaker in meters
   */
  public void angleFromDistance(double distanceToTarget) {
    //Move to point in a line directly at the speaker(6ft off the ground)
    //Then add a small amount of extra angle that scales with distance from the target
    double targetAngle = Constants.LauncherConstants.RADIANS_TO_ENCODER * (Math.atan2(2, distanceToTarget));
    if(targetAngle < Constants.LauncherConstants.LAUNCHER_MAX_ANGLE) {
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