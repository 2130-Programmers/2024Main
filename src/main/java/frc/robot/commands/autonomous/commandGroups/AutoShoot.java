// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.launcher.LaunchNote;
import frc.robot.commands.launcher.StopHandlerMotors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveAndIntakeNote(),
      // new SpinLauncher(RobotContainer.noteHandler, 5000),
      new LaunchNote(RobotContainer.launcherIntake, RobotContainer.launcherWheels),
      new StopHandlerMotors(RobotContainer.launcherAngle, RobotContainer.launcherWheels, RobotContainer.launcherIntake)
    );
  }
}
