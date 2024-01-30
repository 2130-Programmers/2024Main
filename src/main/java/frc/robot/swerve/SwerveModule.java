package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    static double[] drivePowers = new double[4], steerErrors = new double[4];
    static int bestTurnDirAllModules = 0;

    private final int moduleID;
    private CANSparkMax rotationMotor, driveMotor;
    private CANcoder encoder;
    private SwerveVector currentState = new SwerveVector();    

    private boolean shouldFlip = false, stopModule = false;

    /**
     * Constructs a new class to represent an arbitrary swerve module, in our case
     * with two Can SparkMAX
     * This class handles getting the module to the desired angle and setting power,
     * but no kinematics
     * 
     * @param rotationMotor - for now, should be passed an instance of CANSparkMax
     *                      class that represents that motor controller
     * @param driveMotor    - for now, should be passed an instance of CANSparkMax
     *                      class that represents that motor controller
     */
    public SwerveModule(CANSparkMax rotationMotor, CANSparkMax driveMotor, CANcoder encoder, int moduleID) {
        this.moduleID = moduleID;
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
        this.encoder = encoder;
    }

    /**
     * Enable/disable swerve module
     * @param shouldStop - true if module should stop
     */
    public void stopModule(boolean shouldStop) {
        stopModule = shouldStop;
    }

    /**
     * Move module to target angle and calculate power
     * 
     * @param moduleVector
     */
    public void calcDrive(SwerveVector moduleVector) {
        steer(moduleVector.getAngleRadians());
        drivePowers[moduleID] = moduleVector.getMagnitude();

        SmartDashboard.putNumber("Module " + moduleID + " encoder raw", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Module " + moduleID + " encoder position", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Module " + moduleID + " target angle", moduleVector.getAngleDegrees());
    }

    /**
     * Apply calculated power to drive motor
     */
    public void applyDrive() {
        if(!stopModule) {
            driveMotor.set(drivePowers[moduleID]);
        }else{
            driveMotor.set(0);
        }
    }

    /**
     * Calculate and apply steering motor power
     * @param targetSteerAngle - target angle for the module based on swerve kinematics
     */
    private void steer(double targetSteerAngle) {
        int shortestTurnDirection = 0;
        double steerPower = 0, actualTarget;

        //Update module state
        currentState.setAngleRadians((encoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI));

        // Calculate error for steer motor - error is positive if the module should move ccw
        double angleError = targetSteerAngle - currentState.getAngleRadians();
        SmartDashboard.putNumber("Module " + moduleID + " error", angleError);

        // Flip code for rotation ONLY
        // Drivetrain flip handled seperately

        // Decide if this module should flip
        if (Math.abs(angleError) >= Math.PI / 2) {
            shouldFlip = true;
        }else{
            shouldFlip = false;
        }
        
        // Might cause problems when transitioning from a state where it should flip to
        // one where it shouldn't(hopefully not)
        if (shouldFlip) {
            // Flip the target 180 degrees and move it back to within 2pi if it falls outside of that
            actualTarget = (targetSteerAngle + Math.PI) % (Math.PI * 2);
            //Reassign error so that it is only the distance to new desired position
            angleError = actualTarget - currentState.getAngleRadians();
        } else {
            actualTarget = targetSteerAngle;
        }

        //Find shortest move direction
        if (angleError > Constants.DriveTrainConstants.SWERVE_DEADZONE) {// Needs to move cw
            shortestTurnDirection = 1;
        } else if (angleError < -Constants.DriveTrainConstants.SWERVE_DEADZONE) {// Needs to move ccw
            shortestTurnDirection = -1;
        } else {// Stay in same position
            shortestTurnDirection = 0;// Just set to 0 to stop turning
        }

        SmartDashboard.putNumber("Module " + moduleID + " shortestTurnDirection", shortestTurnDirection);

        //Only remaining thing is to synchronise module turn direction!!!

        //Simple proportional feedback loop based on the difference between the
        //module's actual target and current state
        steerPower = Math.abs(angleError) * shortestTurnDirection * 0.05;

        rotationMotor.set(steerPower);
    }

    //If one module power is greater than 1, divide all modules by the greatest magnitude to scale
    public static void scaleMagnitudes() {
        double highMagnitude = 0;

        //Find greatest magnitude
        for (double currentPower : drivePowers) {
            if (currentPower > highMagnitude) {
                highMagnitude = currentPower;
            }

        }

        //If greatest magnitude is greater than one, divide all
        if (highMagnitude > 1) {
            for (int i = 0; i < drivePowers.length; i++) {
                drivePowers[i] = drivePowers[i] / highMagnitude;
            }
        }

    }
}
