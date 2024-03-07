// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyro;

public class AltDriveCom extends Command {
  private DriveTrain driveTrain;
  private Gyro gyro;

  /** Creates a new AltDriveCom. */
  public AltDriveCom(DriveTrain tempDrive, Gyro tempGyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = tempDrive;
    gyro = tempGyro;
    addRequirements(tempDrive);
    addRequirements(tempGyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.altDrive(RobotContainer.driverGamepad.getLeftX(),
                        RobotContainer.driverGamepad.getLeftY(),
                        RobotContainer.driverGamepad.getRightX() * .5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
