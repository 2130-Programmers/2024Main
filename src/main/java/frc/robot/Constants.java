// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0, OPERATOR_PORT = 1;
    public static final double JOYSTICK_DEADZONE = .10;

  }

  public static class DriveTrainConstants {
    public static final double
    BOT_LENGTH = .762,
    BOT_WIDTH = .762,
    SWERVE_DEADZONE = Math.PI/48,
    PEAK_DRIVE_POWER = 1,
    PEAK_TURN_POWER = 1,
    TURN_P_GAIN = .8,
    AUTO_TRANSLATION_GAIN = .25,
    AUTO_ROTATION_GAIN = .1;
    
  }

  public static class VisionConstants {
    public static final Transform3d CAMERA_RELATIVE_TO_ROBOT = new Transform3d(new Translation3d(.381, 0, .4699), new Rotation3d(0, .262, 0));
    public static final Pose3d BLUE_AMP = new Pose3d(), RED_AMP = new Pose3d(), BLUE_SPEAKER = new Pose3d(), RED_SPEAKER = new Pose3d();
  }

  public static class LauncherConstants {
    public static final double
    ENCODER_TO_RADIANS = 0.139105,
    RADIANS_TO_ENCODER = 7.196722,
    LAUNCH_POWER = 1,
    ANGLE_POWER = .35,
    ANGLE_DEADZONE = .1,
    INTAKE_POWER = 1,
    LAUNCHER_MAX_ANGLE = 10;
    
  }
}
