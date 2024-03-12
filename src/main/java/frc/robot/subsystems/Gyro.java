
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  private final ADXRS450_Gyro gyroSensor;

  public Gyro() {
    gyroSensor = new ADXRS450_Gyro();//Initialises with Z axis and 8s calibration time
  }

  /**Average drift from last 8 seconds and apply as drift offset
   */
  public void recalibrateGyro(){
   gyroSensor.calibrate();
  }

  //Reset gyro reading to 0
  public void zeroGyro() {
    gyroSensor.reset();
  }

  /**Returns current angle reading from set axis*/
  public double gyroYaw() {
    return gyroSensor.getAngle();
  }  

  @Override
  public void periodic() {
  }
}
