// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//GITHUB Push Test - Caiden Sutherland - 1/18/24
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.notehandler.*;
import frc.robot.commands.autonomous.InitializeAutonomous;
import frc.robot.commands.autonomous.commandGroups.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.launcher.*;
import frc.robot.commands.vision.*;
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
  public static final LauncherAngle launcherAngle = new LauncherAngle();
  public static final LauncherIntake launcherIntake = new LauncherIntake();
  public static final LauncherWheels launcherWheels = new LauncherWheels();
  public static final Gyro gyro = new Gyro(); 
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final PiVision piVision = new PiVision();
  public static final LimelightVision limelightVision = new LimelightVision();
  public static final PDH pdh = new PDH();


  // --- COMMANDS --- \\\
  //Drivetrain
  private static final TeleDrive teleDriveCommand = new TeleDrive(driveTrain);
  private static final AltDrive altDrive = new AltDrive(driveTrain);
  private static final DriveStraight driveStraight = new DriveStraight(driveTrain);
  //Launcher
  private static final IntakeNote intakeNote = new IntakeNote(launcherIntake);
  private static final ZeroHandler zeroHandler = new ZeroHandler(launcherAngle);
  private static final StopHandlerMotors stopNoteHandlerMotors = new StopHandlerMotors(launcherAngle, launcherWheels, launcherIntake);
  private static final LaunchNote launchNote = new LaunchNote(launcherIntake, launcherWheels);
  private static final SpinLauncher spinLauncher = new SpinLauncher(launcherWheels);
  private static final LauncherToAmp launcherToAmp = new LauncherToAmp(launcherAngle);
  private static final ManualLauncher manualLauncher =  new ManualLauncher(launcherAngle);
  //Vision
  private static final AngleFromAprilTag angleFromAprilTag = new AngleFromAprilTag(launcherAngle, piVision);
  private static final PointAtNote pointAtNote = new PointAtNote(driveTrain, limelightVision);
  //Autonomous
  private static final ShootNoteAndLeave testDriveRoute = new ShootNoteAndLeave();

  // Replace with CommandXboxController or CommandJoystick if needed
    public static final CommandXboxController
      driverGamepad = new CommandXboxController(OperatorConstants.DRIVER_PORT),
      operatorGamepad = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private void configureBindings() {
    //Driver
    driverGamepad.leftBumper().whileTrue(pointAtNote);
    driverGamepad.rightBumper().whileTrue(altDrive);
    driverGamepad.a().whileTrue(driveStraight);
    // driverGamepad.y().whileTrue(point at speaker);


    
    //Operator
    operatorGamepad.leftBumper().onTrue(intakeNote);
    operatorGamepad.rightBumper().onTrue(launchNote);
    operatorGamepad.povUp().onTrue(spinLauncher);
    operatorGamepad.a().onTrue(zeroHandler);
    operatorGamepad.b().whileTrue(stopNoteHandlerMotors);
    operatorGamepad.x().whileTrue(launcherToAmp);
    operatorGamepad.y().whileTrue(angleFromAprilTag);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Set default commands(will run when no other command is using subystem)
    driveTrain.setDefaultCommand(teleDriveCommand);
    // launcherAngle.setDefaultCommand(manualLauncher);
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
