package frc.robot.swerve;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    static double[] drivePowers = new double[4], steerPowers = new double[4], steerErrors = new double[4];
    static int bestTurnDirAllModules = 0;

    private final int moduleID;
    private CANSparkMax rotationMotor, driveMotor;
    private CANcoder encoder;
    private SwerveVector currentState = new SwerveVector();    

    private boolean shouldFlip = true;

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
     * Apply calculated power to drive and rotation motors
     */
    public void applyDrive() {
        driveMotor.set(drivePowers[moduleID]);
        rotationMotor.set(steerPowers[moduleID]);
    }

    /**
     * Move module to target angle and calculate power
     * 
     * @param moduleVector
     */
    public void calcDrive(SwerveVector moduleVector) {
        calcSteer(moduleVector.getAngleRadians());
        drivePowers[moduleID] = moduleVector.getMagnitude();
    }

    /**
     * Calculate steering motor power
     * @param targetSteerAngle - target angle for the module based on swerve kinematics
     */
    private void calcSteer(double targetSteerAngle) {
        int shortestTurnDirection;
        double steerPower, actualTarget;

        //Update module state
        currentState.setAngleRadians((encoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI));

        // Calculate error for steer motor - error is positive if the module should move ccw
        double angleError = targetSteerAngle - currentState.getAngleRadians();
        SmartDashboard.putNumber("Module " + moduleID + "measured error", angleError);

        // Flip code for rotation ONLY
        // Drivetrain flip handled seperately

        // Decide if this module should flip, we use > rather than >= since it's slightly faster to turn the module there than reverse direction
        // if (Math.abs(angleError) > Math.PI / 2) {
        //     shouldFlip = true;
        // }else{
        //     shouldFlip = false;
        // }
        
        // Might cause problems when transitioning from a state where it should flip to
        // one where it shouldn't(hopefully not)
        // if (shouldFlip) {
        //     // Flip the target 180 degrees and move it back to within 2pi if it falls outside of that
        //     actualTarget = (targetSteerAngle + Math.PI) % (Math.PI * 2);
        // } else {
        //     actualTarget = targetSteerAngle;
        // }

        // //Reassign error so that it is only the distance to new desired position
        // angleError = actualTarget - currentState.getAngleRadians();

        SmartDashboard.putNumber("Module " + moduleID + " calculated error", angleError);

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
        // steerPower = Math.abs(angleError) * shortestTurnDirection * 0.25;
        steerPower = angleError * .20;

        steerPowers[moduleID] = steerPower;
    }

    /**
     * Check if each power value is above the limit, and if so set it equal to the limit.
     */
    public static void scaleSteerPowers() {
        //Find greatest magnitude
        for (int i = 0; i < steerPowers.length; i++)) {
            if (steerPowers[i] > Constants.DriveTrainConstants.PEAK_TURN_POWER) steerPowers[i] = Constants.DriveTrainConstants.PEAK_TURN_POWER;
        }
    }


    /**
     * If one or more modules have a drive power greater than the limit, scalle all values so they stay at or below the limit
     */
    public static void scaleMagnitudes() {
        double highMagnitude = 0;

        //Find greatest magnitude
        for (double currentPower : drivePowers) {
            if (currentPower > highMagnitude) highMagnitude = currentPower;
        }
        
        //Calculate a scalar that will make the greatest value equal to the limit
        double scaleMultiplier = Constants.DriveTrainConstants.PEAK_DRIVE_POWER/highMagnitude;

        //If greatest magnitude is greater than the limit, scale all values
        if (highMagnitude > Constants.DriveTrainConstants.PEAK_DRIVE_POWER) {
            for (int i = 0; i < drivePowers.length; i++) {
                drivePowers[i] *= scaleMultiplier;
            }
        }

    }
}
