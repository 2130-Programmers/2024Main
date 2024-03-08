// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  public double pitch;

  
  public static ADXRS450_Gyro gyroSensor;
  
  public Gyro() {
    gyroSensor = new ADXRS450_Gyro(Port.kOnboardCS0);

  }

  //ADIS16470 can only aggregate one axis at a time, switches which axis is being aggregated for angle measurement

  /**Determine which axis the gyro sensor should aggregate
   * 
   * @param axis - either "roll", "yaw", or "pitch" to determine which axis to read
   */

  /*public double balance() {
    if (gyroPitch() >= 0){
      return -.24/(1+Math.pow(Math.E, (.5) * (8.5 -gyroPitch())));
    } else {
      return .17/(1+Math.pow(Math.E, 3 * (14 + gyroPitch())))+.02;      
    } 
  }*/

  //.15/1+e^(18+x) -- more slow fall off centered on -18 still
  //.15/1+e^4(17+x) -- steep fall off on 17%
  //.13/1+e^.7(18+x) + .02 has a lower limit of .02 and a gradual slope at 18
  //.13/1+1.3(18+x) + .02 has a lower limit of .02 and slopes steeper than ^

  /**Average drift from last 8 seconds and apply as drift offset
   */
  public void recalibrateGyro(){
   gyroSensor.calibrate();
  }
  /**Reset gyro reading to 0*/
  public void zeroGyro() {
    gyroSensor.reset();
  }

  /**Returns current angle reading from set axis*/
  public double gyroYaw() {
    return gyroSensor.getAngle();
  }

 /*public static double gyroPitch() {
    return gyroSensor.getYComplementaryAngle(); */ 

  

  @Override
  public void periodic() {
  }
}

