// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PiVision;
import frc.robot.subsystems.notehandler.LauncherWheels;

public class LaunchPowerFromAprilTag extends Command {
  private final LauncherWheels launcherWheels;
  private final PiVision vision;
  /** Creates a new LaunchPowerFromAprilTag. */
  public LaunchPowerFromAprilTag(LauncherWheels launcherWheels, PiVision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcherWheels, vision);
    this.launcherWheels = launcherWheels;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherWheels.setLaunchPower(vision.getDistanceToTarget() * .08 + .2);
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
