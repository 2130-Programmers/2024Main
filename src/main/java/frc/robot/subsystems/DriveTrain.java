// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.*;

public class DriveTrain extends SubsystemBase {
  private SwerveModule
  flModule = new SwerveModule(new CANSparkMax(0, MotorType.kBrushless), new CANSparkMax(0, MotorType.kBrushless), new CANcoder(0), 0),
  frModule = new SwerveModule(new CANSparkMax(0, MotorType.kBrushless), new CANSparkMax(0, MotorType.kBrushless), new CANcoder(0), 1),
  blModule = new SwerveModule(new CANSparkMax(0, MotorType.kBrushless), new CANSparkMax(0, MotorType.kBrushless), new CANcoder(0), 2),
  brModule = new SwerveModule(new CANSparkMax(0, MotorType.kBrushless), new CANSparkMax(0, MotorType.kBrushless), new CANcoder(0), 3);

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  //Asssuming robot is square
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


    //Vectors to pass to the swerve modules every loop
    SwerveVector flVector, frVector, blVector, brVector;

    //Vectors or lines from the origin to each swerve module. We can later change the origin to allow rotation around points other than the center of the robot
    final SwerveVector
    frPositionVector = new SwerveVector(Math.PI/4, moduleDistFromCenter),
    flPositionVector = new SwerveVector((Math.PI*3)/4, moduleDistFromCenter),
    blPositionVector = new SwerveVector((Math.PI*5)/4, moduleDistFromCenter),
    brPositionVector = new SwerveVector((Math.PI*7)/4, moduleDistFromCenter);

    //A vector perpendicular to the positionVector of each module, with a strength proportional to how much we want the robot to turn
    //This will be the rotation component of each module's calculation
    final SwerveVector 
    frRotationVector = new SwerveVector(frPositionVector.getAngleRadians() + Math.PI/2, r),
    flRotationVector = new SwerveVector(flPositionVector.getAngleRadians() + Math.PI/2, r),
    blRotationVector = new SwerveVector(blPositionVector.getAngleRadians() + Math.PI/2, r),
    brRotationVector = new SwerveVector(brPositionVector.getAngleRadians() + Math.PI/2, r);

    //Convert joystick inputs to polar/vector
    //This is the translational component of our swerve inputs.
    if(x == 0) x = 0.0001;//Don't divide by 0
    SwerveVector translationVector = new SwerveVector(Math.atan2(y, x), Math.sqrt((x * x) + (y * y)));

    //Combine the translation and rotation components for each module
    frVector = SwerveVector.combineVectors(translationVector, frRotationVector);
    flVector = SwerveVector.combineVectors(translationVector, flRotationVector);
    brVector = SwerveVector.combineVectors(translationVector, brRotationVector);
    blVector = SwerveVector.combineVectors(translationVector, blRotationVector);

    flModule.calcDrive(flVector);
    frModule.calcDrive(frVector);
    blModule.calcDrive(blVector);
    brModule.calcDrive(brVector);

    SwerveModule.scaleMagnitudes();

    flModule.applyDrive();
    frModule.applyDrive();
    blModule.applyDrive();
    brModule.applyDrive();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
