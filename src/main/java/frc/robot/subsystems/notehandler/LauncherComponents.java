// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notehandler;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public final class LauncherComponents {

    //Set up inputs for safety prox and note detection switch
    protected static final DigitalInput noteLimitSwitch = new DigitalInput(1);

    //Define rotation and launch motors
    protected static final TalonFX
    launcherTop = new TalonFX(33),
    launcherBottom = new TalonFX(32),
    rotateLeft = new TalonFX(30),
    rotateRight = new TalonFX(31);

    //Define intake motors
    protected static final CANSparkMax
    intakeTop = new CANSparkMax(35, MotorType.kBrushless),
    intakeBottom = new CANSparkMax(34, MotorType.kBrushless);

    public LauncherComponents() {
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
}
