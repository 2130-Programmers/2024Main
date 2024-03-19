// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase {
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry xError = limelightTable.getEntry("tx"), targetArea = limelightTable.getEntry("ta");
  /** Creates a new LimelightVision. */
  public LimelightVision() {}

  /**
   * Get the angle of the note relative to a line perpendicular to the center of the rear frame of the bot.
   * Returns 0 if nothing is found
   * @return angle in degrees
   */
  public double angleToNearestNote() {
    return xError.getDouble(0);
  }

  /**
   * Get the percentage of the cameras vision field taken up by the target
   */
  public double getTargetArea() {
    return targetArea.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
