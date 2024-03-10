// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//GITHUB Push Test - Caiden Sutherland - 1/18/24
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ManualLauncher;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // --- SUBSYSTEMS --- \\\
  public static final NoteHandler noteHandler = new NoteHandler();
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final Vision vision = new Vision();

  // --- COMMANDS --- \\\
  private static final ManualLauncher manualLauncher = new ManualLauncher(noteHandler);
  public static final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driveTrain);

  // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final CommandXboxController
      driverGamepad = new CommandXboxController(OperatorConstants.DRIVER_PORT),
      operatorGamepad = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Set default commands(will run when no other command is using subystem)
    driveTrain.setDefaultCommand(teleDriveCommand);
    noteHandler.setDefaultCommand(manualLauncher);
  }


  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
