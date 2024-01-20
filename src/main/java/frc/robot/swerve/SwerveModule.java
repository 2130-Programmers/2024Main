package frc.robot.swerve;

import com.revrobotics.CANSparkMax;

public class SwerveModule {
    static double[] drivePowers = new double[4];

    private final int moduleID;
    private CANSparkMax rotationMotor, driveMotor;
    private SwerveVector currentState = new SwerveVector();

    private double drivePower, steerPower;
    
    /**
     * Constructs a new class to represent an arbitrary swerve module, in our case with two Can Spark MAX
     * This class handles getting the module to the desired angle and setting power, but no kinematics
     * @param rotationMotor - for now, should be passed an instance of CANSparkMax class that represents that motor controller
     * @param driveMotor - for now, should be passed an instance of CANSparkMax class that represents that motor controller
     */
    public SwerveModule(CANSparkMax rotationMotor, CANSparkMax driveMotor, int moduleID) {
        this.moduleID = moduleID;
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
    }

    /**
     * Move module to target angle and set power
     * @param moduleVector
     */
    public void drive(SwerveVector moduleVector) {
        //Calculate error for steer motor
        double angleError = SwerveVector.subVectorAngles(currentState, moduleVector);

        //Store 
        drivePowers[moduleID] = moduleVector.getMagnitude();

        steerPower = angleError * .01;
    }
}
