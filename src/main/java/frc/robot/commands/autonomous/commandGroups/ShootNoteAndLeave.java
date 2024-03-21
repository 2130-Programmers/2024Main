// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.AutoSpeakerAim;
import frc.robot.commands.autonomous.GoToLocation;
import frc.robot.commands.launcher.LaunchNote;
import frc.robot.commands.vision.PointAtSpeaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteAndLeave extends SequentialCommandGroup {
  /** Creates a new TestDriveRoute. */
  public ShootNoteAndLeave() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new GoToLocation(RobotContainer.driveTrain, new Pose2d(0, 0, new Rotation2d(Math.PI))),
      new PointAtSpeaker(RobotContainer.piVision, RobotContainer.driveTrain),
      new AutoSpeakerAim(RobotContainer.launcherAngle, RobotContainer.piVision),
      new LaunchNote(RobotContainer.launcherIntake, RobotContainer.launcherWheels),
      new GoToLocation(RobotContainer.driveTrain, new Pose2d(2, 5, new Rotation2d(Math.PI)))
    );
  }
}
