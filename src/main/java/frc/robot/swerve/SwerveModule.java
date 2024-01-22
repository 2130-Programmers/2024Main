package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class SwerveModule {
    static double[] drivePowers = new double[4], steerErrors = new double[4];
    static int bestTurnDirAllModules = 0;

    private final int moduleID;
    private CANSparkMax rotationMotor, driveMotor;
    private CANcoder encoder;
    private SwerveVector currentState = new SwerveVector();

    private boolean shouldFlip;

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
     * Move module to target angle and calculate power
     * 
     * @param moduleVector
     */
    public void drive(SwerveVector moduleVector) {
        steer(moduleVector.getAngleRadians());
        drivePowers[moduleID] = moduleVector.getMagnitude();
    }

    /**
     * Apply calculated power to drive motor
     */
    public void applyDrive() {
        driveMotor.set(drivePowers[moduleID]);
    }

    /**
     * Calculate and apply steering motor power
     * @param targetSteerAngle - target angle for the module based on swerve kinematics
     */
    private void steer(double targetSteerAngle) {
        int shortestTurnDirection = 0;
        double steerPower = 0, actualTarget;

        //Update module state
        currentState.setAngleDegrees(encoder.getPosition().getValueAsDouble());

        // Calculate error for steer motor
        double angleError = currentState.getAngleRadians() - targetSteerAngle;

        // Flip code for rotation ONLY
        // Drivetrain flip handled seperately

        // Decide if this module should flip
        if (Math.abs(angleError) >= Math.PI / 2)
            shouldFlip = true;
        else
            shouldFlip = false;

        // Might cause problems when transitioning from a state where it should flip to
        // one where it shouldn't(hopefully not)
        if (shouldFlip) {
            // Flip the target 180 degrees and move it back to within 2pi if it falls outside of that
            actualTarget = (targetSteerAngle + Math.PI) % (Math.PI * 2);
        } else {
            actualTarget = targetSteerAngle;
        }

        //Store error to desired position

        if (angleError > Constants.DriveTrainConstants.SWERVE_DEADZONE) {// Needs to move cw
            shortestTurnDirection = 1;
        } else if (angleError < -Constants.DriveTrainConstants.SWERVE_DEADZONE) {// Needs to move ccw
            shortestTurnDirection = -1;
        } else {// Stay in same position
            shortestTurnDirection = 0;// Just set to 0 to stop turning
        }

        // Simple proportional feedback loop based on the difference between the
        // module's actual target and current state
        steerPower = Math.abs(currentState.getAngleRadians() - actualTarget) * shortestTurnDirection * 0.05;

        rotationMotor.set(steerPower);
    }

    public static void scaleMagnitudes() {
        double highMagnitude = 0;
        for (double currentPower : drivePowers) {
            if (currentPower > highMagnitude) {
                highMagnitude = currentPower;
            }

        }
        if (highMagnitude > 1) {
            for (int i = 0; i < drivePowers.length; i++) {
                drivePowers[i] = drivePowers[i] / highMagnitude;
            }
        }

    }
}
