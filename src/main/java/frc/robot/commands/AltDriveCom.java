// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */


package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyro;

public class AltDriveCom extends Command {
  private DriveTrain driveTrain;
  private Gyro gyro;

  /** Creates a new AltDriveCom. */
  public AltDriveCom(DriveTrain tempdrive, Gyro tempgyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = tempdrive;
    gyro = tempgyro;
    this.addRequirements(tempdrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveTrain.moveSwerveAxisAlt(RobotContainer.driverJoy.getRawAxis(Constants.LeftAxisX),
                                                RobotContainer.driverJoy.getRawAxis(Constants.LeftAxisY),
                                                RobotContainer.driverJoy.getRawAxis(Constants.RightAxisX) * .5,
                                                .5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.driverJoy.getRightBumperPressed();
  }
}
