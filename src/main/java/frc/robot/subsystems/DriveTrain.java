// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.*;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  //Asssuming robot is square
  private static final double moduleDistFromCenter = Math.sqrt(Math.pow(Constants.DriveTrainConstants.BOT_LENGTH, 2) * 2);

  public void calculateKinematics(double x, double y, double r) {
    
    if(x == 0) x = 0.0001;//Don't divide by 0

    //Convert joystick inputs to polar/vector
    SwerveVector movementVector = new SwerveVector(Math.atan(y/x), Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));

    SwerveModule flModule, frModule, blModule, brModule;

    SwerveVector flVector, frVector, bVectorl, brVector;

    SwerveVector flPositionVector = new SwerveVector(Math.PI/4, moduleDistFromCenter);
    /*
     * Simply put, swerve takes a vector for velocity and an angular rotation component as inputs.
     * This function simply handles passing target values to each module(solving kinematics but not coordinating fip/rotations)
     * 
     * The origin(0, 0) on our 2d coordinate plane is found at the center of the robot frame
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

     flVector = new SwerveVector()
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
