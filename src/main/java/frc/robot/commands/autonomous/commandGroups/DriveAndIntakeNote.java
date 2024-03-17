// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveAtNote;
import frc.robot.commands.launcher.IntakeNote;
import frc.robot.commands.vision.PointAtNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndIntakeNote extends ParallelDeadlineGroup {
  /** Creates a new DriveAndIntakeNote. */
  public DriveAndIntakeNote() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeNote(RobotContainer.launcherIntake));
    
    addCommands(
      new PointAtNote(RobotContainer.driveTrain, RobotContainer.limelightVision),
      new DriveAtNote(RobotContainer.driveTrain)
    );
  }
}
