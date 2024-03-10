// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NoteHandler;

public class ManualLauncher extends Command {
  NoteHandler noteHandler;
  /** Creates a new ManualLauncher. */
  public ManualLauncher(NoteHandler tempSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    noteHandler = tempSub;
    addRequirements(tempSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteHandler.setLaunchPower(RobotContainer.operatorGamepad.getRightTriggerAxis());
    noteHandler.setIntakePower(RobotContainer.operatorGamepad.getLeftTriggerAxis());
    noteHandler.setRotatePower(RobotContainer.operatorGamepad.getLeftY());
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
