package frc.robot.swerve;

import com.revrobotics.CANSparkMax;

public class SwerveModule {
    private CANSparkMax rotationMotor, driveMotor;
    
    /**
     * Constructs a new class to represent an arbitrary swerve module, in our case with two Can Spark MAX
     * @param rotationMotor - for now, should be passed an instance of CANSparkMax class that represents that motor controller
     * @param driveMotor - for now, should be passed an instance of CANSparkMax class that represents that motor controller
     */
    public SwerveModule(CANSparkMax rotationMotor, CANSparkMax driveMotor) {
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
    }

    public void drive(SwerveVector moduleVector) {

    }
}
