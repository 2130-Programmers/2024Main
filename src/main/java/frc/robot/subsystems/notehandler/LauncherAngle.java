// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notehandler;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class LauncherAngle extends PIDSubsystem {
  /** Creates a new LauncherAngle. */

  //Feedforward controller to predict required power for arm angle
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, .85, 3);

  //Array to store the angles for the launcher and associated distances
  //Each index indicates the distance from the speaker in feet
  private double[] launchAngles;

  //File reader object
  private BufferedReader fileReader;

  public LauncherAngle() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1.5, 0, .05));
      try {
        fileReader = new BufferedReader(new FileReader(Filesystem.getDeployDirectory().getAbsolutePath() + "/launcherPositions.csv"));
      } catch (FileNotFoundException fileNotFoundException) {
        System.out.println("Could not open launcher setpoint file");
        fileNotFoundException.printStackTrace();
      }

      try {
        launchAngles = new double[20];

        String line;
        while((line = fileReader.readLine()) != null) {
          String[] elements;
          
          //We only keep the second element in each row, distances are just there for human readability
          elements = line.split(",");
          launchAngles[Integer.parseInt(elements[0])] = Double.parseDouble(elements[1]);
        }
      } catch(IOException ioException) {
        System.out.println("Error reading launcher calibration file after openeing");
        ioException.printStackTrace();
      }

      for(int i = 0; i < launchAngles.length; i++) {
        System.out.println("Setpoint is " + launchAngles[i] + " at distance " + i);
      }
  }

  /**
   * Allow the launcher to coast to its resting point
   */
  public void stopAngleMotors() {
    LauncherComponents.rotateLeft.set(0);
    LauncherComponents.rotateRight.set(0);
  }

  /**
   * Move the launcher to specified angle: higher values move the intake higher, and thus the launcher points lower. 0 is horizontal, -8 is the ground
   * @param angle - the angle to move the launcher to, expressed in encoder ticks
   */
  public void moveArmFeedForward(double position, double velocity) {
    //We need basically 1v to get the launcher to move slightly
    double feedforwardValue = armFeedforward.calculate(position, velocity);
    setRotateVoltage(feedforwardValue);
  }

  /**
   * Move the launcher to the proper angle for shooting froma specific distance
   * @param distanceToTarget - the distance from the launcher to the speaker in meters
   */
  public void angleFromDistance(double distanceToTarget) {
    //Move to point in a line directly at the speaker(6ft off the ground)
    //Then add a small amount of extra angle that scales with distance from the target
    if(distanceToTarget >= launchAngles.length - 1) {
      stopAngleMotors();
    } else {
      setSetpoint(launchAngles[(int)Math.round(distanceToTarget)]);
    }
  }

  /**
   * Set the voltage supplied to the angle control motors
   * @param volts - the value in volts, should not exceed -12 - 12
   */
  public void setRotateVoltage(double volts) {
    LauncherComponents.rotateLeft.setControl(new VoltageOut(volts));
    LauncherComponents.rotateRight.setControl(new VoltageOut(volts));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    MathUtil.clamp(output, -0.25, 0.5);
    // Use the output here
    moveArmFeedForward(setpoint, output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double encoderAngle = (LauncherComponents.rotateLeft.getPosition().getValueAsDouble() + LauncherComponents.rotateRight.getPosition().getValueAsDouble())/2;
    return encoderAngle * Constants.LauncherConstants.ENCODER_TO_RADIANS;
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), getSetpoint());
    }
    SmartDashboard.putNumber("Launcher angle", (LauncherComponents.rotateLeft.getPosition().getValueAsDouble() + LauncherComponents.rotateRight.getPosition().getValueAsDouble())/2);
  }
}
