// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//GITHUB Push Test - Caiden Sutherland - 1/18/24
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.notehandler.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.launcher.*;
import frc.robot.commands.vision.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // --- SUBSYSTEMS --- \\\
  public static final LauncherAngle launcherAngle = new LauncherAngle();
  public static final LauncherIntake launcherIntake = new LauncherIntake();
  public static final LauncherWheels launcherWheels = new LauncherWheels();
  public static final Gyro gyro = new Gyro(); 
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final PiVision piVision = new PiVision();
  public static final LimelightVision limelightVision = new LimelightVision();


  // --- COMMANDS --- \\\
  //Drivetrain
  private static final TeleDriveCommand teleDriveCommand = new TeleDriveCommand(driveTrain);
  private static final AltDrive altDrive = new AltDrive(driveTrain);
  //Launcher
  private static final IntakeNote intakeNote = new IntakeNote(launcherIntake);
  private static final ZeroHandler zeroHandler = new ZeroHandler(launcherAngle);
  private static final StopHandlerMotors stopNoteHandlerMotors = new StopHandlerMotors(launcherAngle, launcherWheels, launcherIntake);
  private static final LauncherToAmp moveToAmp = new LauncherToAmp(launcherAngle);
  private static final LaunchNote launchNote = new LaunchNote(launcherIntake, launcherWheels);
  private static final SpinLauncher spinLauncher = new SpinLauncher(launcherWheels);
  private static final LauncherToAmp launcherToAmp = new LauncherToAmp(launcherAngle);
  //Vision
  private static final AngleFromAprilTag angleFromAprilTag = new AngleFromAprilTag(launcherAngle, piVision);
  private static final LaunchPowerFromAprilTag launchPowerFromAprilTag = new LaunchPowerFromAprilTag(launcherWheels, piVision);
  private static final PointAtNote pointAtNote = new PointAtNote(driveTrain, limelightVision);


  // Replace with CommandXboxController or CommandJoystick if needed
    public static final CommandXboxController
      driverGamepad = new CommandXboxController(OperatorConstants.DRIVER_PORT),
      operatorGamepad = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private void configureBindings() {
    driverGamepad.leftBumper().whileTrue(pointAtNote);
    driverGamepad.rightBumper().whileTrue(altDrive);
    driverGamepad.x().onTrue(intakeNote);
    driverGamepad.a().whileTrue(angleFromAprilTag);
    driverGamepad.y().onTrue(zeroHandler);
    driverGamepad.start().onTrue(spinLauncher);
    driverGamepad.back().onTrue(launchNote);
    driverGamepad.b().onTrue(stopNoteHandlerMotors);
    driverGamepad.povUp().onTrue(launcherToAmp);
    driverGamepad.povDown().onTrue(zeroHandler);
    driverGamepad.povRight().onTrue(Commands.sequence(spinLauncher, launchNote));

    operatorGamepad.b().onTrue(zeroHandler);
    operatorGamepad.leftTrigger().onTrue(launchNote);
    operatorGamepad.leftBumper().onTrue(stopNoteHandlerMotors);
    operatorGamepad.povDown().whileTrue(moveToAmp);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Set default commands(will run when no other command is using subystem)
    driveTrain.setDefaultCommand(teleDriveCommand);
    // noteHandler.setDefaultCommand(stopNoteHandlerMotors);
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
