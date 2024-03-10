package frc.robot.swerve;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    static double[] drivePowers = new double[4], steerPowers = new double[4];
    static int bestTurnDirAllModules = 0;

    private final int moduleID;
    private CANSparkMax rotationMotor, driveMotor;
    private CANcoder encoder;
    private SwerveVector currentState = new SwerveVector();    

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
     * @param encoder       - accepts an instance of cancoder for measuring module rotation
     * @param moduleID      - unique integer identifier, should start with 0 and increment sequentially for each module
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
    public void applyMotorPowers() {
        driveMotor.set(drivePowers[moduleID]);
        rotationMotor.set(steerPowers[moduleID]);
    }

    /**
     * Calculate steering motor power
     * @param moduleVector - vector with target magnitude and angle for swerve module
     */
    public void calcDrive(SwerveVector moduleVector) {
        //Update module state
        currentState.setAngleRadians((encoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI));

        //Apply initial drivce power
        drivePowers[moduleID] = moduleVector.getMagnitude();

        // Calculate error for steer motor - error is positive if the module should move ccw.
        // These methods automatically correct for the inneffeciency that arises to avoid turning 270 one way instead of 90 the other
        double rawAngleError = SwerveVector.getClosestAngle(moduleVector, currentState);
        double alternateAngleError = SwerveVector.getAlternateAngle(moduleVector, currentState);

        if(Math.abs(rawAngleError) < Math.abs(alternateAngleError)) {
            steerPowers[moduleID] = rawAngleError * Constants.DriveTrainConstants.TURN_P_GAIN;
            SmartDashboard.putNumber("Module " + moduleID + " used error", rawAngleError);
            drivePowers[moduleID] *= Math.abs(Math.cos(rawAngleError));
        } else {
            steerPowers[moduleID] = alternateAngleError * Constants.DriveTrainConstants.TURN_P_GAIN;
            SmartDashboard.putNumber("Module " + moduleID + " used error", alternateAngleError);
            drivePowers[moduleID] *= -Math.abs(Math.cos(alternateAngleError));
        }
    }

    /**
     * Check if each power value is above the limit, and if so set it equal to the limit.
     */
    public static void scaleSteerPowers() {
        //Find greatest magnitude
        for (int i = 0; i < steerPowers.length; i++) {
            if (Math.abs(steerPowers[i]) > Constants.DriveTrainConstants.PEAK_TURN_POWER) steerPowers[i] = (Constants.DriveTrainConstants.PEAK_TURN_POWER) * (Math.abs(steerPowers[i])/steerPowers[i]);
        }
    }


    /**
     * If one or more modules have a drive power greater than the limit, scalle all values so they stay at or below the limit
     */
    public static void scaleMagnitudes() {
        double highMagnitude = 0;

        //Find greatest magnitude
        for (double currentPower : drivePowers) {
            if (Math.abs(currentPower) > Math.abs(highMagnitude)) highMagnitude = currentPower;
        }
        
        //Calculate a scalar that will make the greatest value equal to the limit
        double scaleMultiplier = Constants.DriveTrainConstants.PEAK_DRIVE_POWER/Math.abs(highMagnitude);

        //If greatest magnitude is greater than the limit, scale all values
        if (Math.abs(highMagnitude) > Constants.DriveTrainConstants.PEAK_DRIVE_POWER) {
            for (int i = 0; i < drivePowers.length; i++) {
                drivePowers[i] *= scaleMultiplier;
            }
        }
    }
}
