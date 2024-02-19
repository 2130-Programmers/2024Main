// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public NoteHandler() {}

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

  private void setIntakePower(double power) {
    intakeTop.set(power);
    intakeBottom.set(power);
  }

  public void setLaunchPower(double power) {
    launcherTop.set(power);
    launcherBottom.set(power);
  }

  private void setRotatePower(double power) {
    rotateLeft.set(power);
    rotateRight.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
