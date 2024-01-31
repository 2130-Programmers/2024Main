package frc.robot.swerve;

public class SwerveVector {
    private double direction, power;
    
    /**
     * A vector to be passed to a swerve module, contains direction and speed
     * @param direction - value in radians from 0 to 2pi
     * @param power - power value
     */
    public SwerveVector(double direction, double power) {
        this.direction = direction % Math.PI*2;
        this.power = power;
    }

    /**
     * A vector to be passed to a swerve module, power and direction at 0
     */
    public SwerveVector() {
        direction = 0;
        power = 0;
    }


    /**
     * Find the difference in angles of two vectors
     * @param minuend - the angle to be subtracted from
     * @param subtrahend - the angle we subtract
     * @return the difference between the two angles
     */
    public static double subVectorAngles(SwerveVector minuend, SwerveVector subtrahend) {
        return minuend.getAngleRadians() - subtrahend.getAngleRadians();
    }

    /**
     * Combine two vectors. Angle is capped to 2pi, after that starts over on a new revolution.
     * @param a - first vector
     * @param b - second vector
     * @return a new SwerveVector with angle(limited) and power(unclamped)
     */
    public static SwerveVector combineVectors(SwerveVector a, SwerveVector b) {
        /*
         * This function works by 
         */

        //Decompose vectors to cartesian coordinates
        double
        aX = a.getMagnitude() * Math.cos(a.getAngleRadians()), aY = a.getMagnitude() * Math.sin(a.getAngleRadians()),
        bX = b.getMagnitude() * Math.cos(b.getAngleRadians()), bY = b.getMagnitude() * Math.sin(b.getAngleRadians());

        //Add cartesian coordinates
        double mX = aX + bX, mY = aY + bY;

        //Convert to new vector with direction and magnitude
        return new SwerveVector(Math.atan(mY/mX), Math.sqrt((mX * mX) + (mY * mY)));
    }

    /**
     * Add to angle of vector
     * @param radsToAdd - amount in radians to add
     */
    public void addRadsToAngle(double radsToAdd) {
        direction += radsToAdd;
        direction = direction % (Math.PI/2);
    }

    /**
     * Multiply the vector by value
     * @param multBy - the value to multiply the vector by
     */
    public void multiplyVector(double multBy) {
        power *= multBy;
    }

    /**
     * Gets the vector angle in degrees
     * @return angle in degrees from 0 to 360
     */
    public double getAngleDegrees(){
        return direction * 57.29578;
    }

    /**
     * Gets the vector angle in radians
     * @return angle in degrees from 0 to 2pi
     */
    public double getAngleRadians(){
        return direction;
    }

    /**
     * Sets the vector angle in degrees
     */
    public void setAngleDegrees(double angle){
        direction = (angle / 57.29578) % (Math.PI/2);
    }

    /**
     * Sets the vector angle in degrees
     */
    public void setAngleRadians(double angle){
        direction = angle % (Math.PI/2);
    }

    /**
     * Returns the power of the vector
     * @return vector power, from 0 to 1
     */
    public double getMagnitude() {
        return power;
    }

    /**
     * Sets the power of the vector
     * @param setPower - power for the vector, must be from 0 to 1
     */
    public void setPower(double power) {
        this.power = power;
    }
}
