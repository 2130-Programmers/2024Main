// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.*;

public class DriveTrain extends SubsystemBase {
  private static final CANSparkMax
  flRotationMotor = new CANSparkMax(12, MotorType.kBrushless),
  frRotationMotor = new CANSparkMax(10, MotorType.kBrushless),
  blRotationMotor = new CANSparkMax(14, MotorType.kBrushless),
  brRotationMotor = new CANSparkMax(16, MotorType.kBrushless),
  flDriveMotor = new CANSparkMax(11, MotorType.kBrushless),
  frDriveMotor = new CANSparkMax(13, MotorType.kBrushless),
  blDriveMotor = new CANSparkMax(15, MotorType.kBrushless),
  brDriveMotor = new CANSparkMax(17, MotorType.kBrushless);

  private static final CANcoder
  flEncoder = new CANcoder(20),
  frEncoder = new CANcoder(21),
  blEncoder = new CANcoder(22),
  brEncoder = new CANcoder(23);

  private static SwerveModule flModule, frModule, blModule, brModule;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    flRotationMotor.setInverted(true);
    frRotationMotor.setInverted(true);
    blRotationMotor.setInverted(true);
    brRotationMotor.setInverted(true);
    
    flDriveMotor.setInverted(false);
    frDriveMotor.setInverted(false);
    blDriveMotor.setInverted(false);
    brDriveMotor.setInverted(false);

    flModule = new SwerveModule(flRotationMotor, flDriveMotor, flEncoder, 0);
    frModule = new SwerveModule(frRotationMotor, frDriveMotor, frEncoder, 1);
    blModule = new SwerveModule(blRotationMotor, blDriveMotor, blEncoder, 2);
    brModule = new SwerveModule(brRotationMotor, brDriveMotor, brEncoder, 3);
  }

  //Assuming robot is square
  private static final double moduleDistFromCenter = Math.sqrt(Math.pow(Constants.DriveTrainConstants.BOT_LENGTH, 2) * 2);

  public void calculateKinematics(double x, double y, double r) {
      /*
      * Simply put, swerve takes a vector for velocity and an angular rotation component as inputs.
      * This function simply handles passing target values to each module(solving kinematics but not coordinating fip/rotations)
      * 
      * The origin(0, 0) on our 2d coordinate plane is found at the center of the robot frame
      * By shifting the origin or the positions of the modules relative to the origin we can change the point about which the robot rotates
      * This coordinate plane uses inches as units(currently)
      * 
      * Module locations are expressed as coordinates on that plane
      * Our implementation is designed expecting four modules, but could be scaled down to three or up to solve for more than four modules
      * 
      * Fundamentally, each module needs to probel itself(and the attatched robot) along a vector which can be described rather simply.
      * We do this by adding together the translation and rotational components of our inputs and multiplying that sum by a vector perpendicular to the line from origin to module n.
      * 
      * Good resource that covers what is done here: https://dominik.win/blog/programming-swerve-drive/
      */

      x = (Math.abs(x) < Constants.OperatorConstants.JOYSTICK_DEADZONE) ? 0 : x;
      y = (Math.abs(y) < Constants.OperatorConstants.JOYSTICK_DEADZONE) ? 0 : y;
      r = (Math.abs(r) < Constants.OperatorConstants.JOYSTICK_DEADZONE) ? 0 : r;

      //Vectors to pass to the swerve modules every loop
      SwerveVector flVector, frVector, blVector, brVector;

      //Vectors or lines from the origin to each swerve module. We can later change the origin to allow rotation around points other than the center of the robot
      final SwerveVector
      frPositionVector = new SwerveVector(Math.PI/4, moduleDistFromCenter),
      flPositionVector = new SwerveVector(Math.PI*3/4, moduleDistFromCenter),
      blPositionVector = new SwerveVector(Math.PI*5/4, moduleDistFromCenter),
      brPositionVector = new SwerveVector(Math.PI*7/4, moduleDistFromCenter);

      //A vector perpendicular to the positionVector of each module, with a strength proportional to how much we want the robot to turn
      //This will be the rotation component of each module's calculation
      final SwerveVector
      frRotationVector = new SwerveVector(frPositionVector.getAngleRadians() + Math.PI/2, r),
      flRotationVector = new SwerveVector(flPositionVector.getAngleRadians() + Math.PI/2, r),
      blRotationVector = new SwerveVector(blPositionVector.getAngleRadians() + Math.PI/2, r),
      brRotationVector = new SwerveVector(brPositionVector.getAngleRadians() + Math.PI/2, r);

      //Convert joystick inputs to polar/vector
      //This is the translational component of our swerve inputs.
      SmartDashboard.putNumber("safe atan", SwerveVector.safeAtan2(y, x));
      SwerveVector translationVector = new SwerveVector(SwerveVector.safeAtan2(y, x), Math.sqrt((x * x) + (y * y)));
      
      //Combine the translation and rotation components for each module
      flVector = SwerveVector.combineVectors(translationVector, flRotationVector);
      frVector = SwerveVector.combineVectors(translationVector, frRotationVector);
      blVector = SwerveVector.combineVectors(translationVector, blRotationVector);
      brVector = SwerveVector.combineVectors(translationVector, brRotationVector);

      flModule.calcDrive(flVector);
      frModule.calcDrive(frVector);
      blModule.calcDrive(blVector);
      brModule.calcDrive(brVector);

      SwerveModule.scaleMagnitudes();
      // SwerveModule.scaleSteerPowers();

      flModule.applyMotorPowers();
      frModule.applyMotorPowers();
      blModule.applyMotorPowers();
      brModule.applyMotorPowers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("fL Encoder", flEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("fr Encoder", frEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("bl Encoder", blEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("br Encoder", brEncoder.getAbsolutePosition().getValueAsDouble());
  }
}
