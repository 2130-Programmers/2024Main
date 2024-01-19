package frc.robot.swerve;

import com.revrobotics.CANSparkMax;

public class SwerveModule {
    private CANSparkMax rotationMotor, driveMotor;
    private SwerveVector currentState = new SwerveVector();
    
    /**
     * Constructs a new class to represent an arbitrary swerve module, in our case with two Can Spark MAX
     * This class handles getting the module to the desired angle and setting power, but no kinematics
     * @param rotationMotor - for now, should be passed an instance of CANSparkMax class that represents that motor controller
     * @param driveMotor - for now, should be passed an instance of CANSparkMax class that represents that motor controller
     */
    public SwerveModule(CANSparkMax rotationMotor, CANSparkMax driveMotor) {
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
    }

    /**
     * Move module to target angle and set power
     * @param moduleVector
     */
    public void drive(SwerveVector moduleVector) {
        double angleError = SwerveVector.compareVectorAngles(currentState, moduleVector);
    }
}
