package frc.robot.swerve;

public class SwerveVector {
    private double direction, power;
    
    /**
     * A vector to be passed to a swerve module, contains direction and speed
     * @param direction - value in radians from 0 to 2pi
     * @param power - power value
     */
    public SwerveVector(double direction, double power) {
        this.direction = direction%(Math.PI*2);
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
     * Convert the values from atan(-pi to pi) to a system using 0 to 2pi
     * @param y - y value
     * @param x - x value
     * @return a value from 0 to 2pi
     */
    public static double safeAtan2(double y, double x) {
        double atanValue = Math.atan2(y, x);
        return atanValue >= 0 ? atanValue : atanValue + Math.PI*2;
    }


    /**
     * Find the error between target and current location
     * @param minuend - the angle to be subtracted from
     * @param subtrahend - the angle we subtract
     * @return the difference between the two angles
     */
    public static double getClosestAngle(SwerveVector minuend, SwerveVector subtrahend) {
        //Find the error from -pi to pi and scale to proper domain
        return convertToAltDomain((minuend.getAngleRadians() - subtrahend.getAngleRadians()) % (2 * Math.PI));
    }

    /**
     * Find the difference between target and current, but try an alternate location that is offset 180 degrees
     * @param minuend - the angle to be subtracted from
     * @param subtrahend - the angle we subtract
     * @return the difference between the two angles
     */
    public static double getAlternateAngle(SwerveVector minuend, SwerveVector subtrahend) {
        //Find the error from -pi to pi, then add 180 to that value and scale it back to the proper domain
        return convertToAltDomain((minuend.getAngleRadians() + Math.PI - subtrahend.getAngleRadians()) % (Math.PI * 2));
    }

    /**
     * Combine two vectors. Angle is capped to 2pi, after that starts over on a new revolution.
     * @param a - first vector
     * @param b - second vector
     * @return a new SwerveVector with angle(limited) and power(unclamped)
     */
    public static SwerveVector combineVectors(SwerveVector a, SwerveVector b) {

        //Decompose vectors to cartesian coordinates
        double
        aX = a.getMagnitude() * Math.cos(a.getAngleRadians()),
        aY = a.getMagnitude() * Math.sin(a.getAngleRadians()),
        bX = b.getMagnitude() * Math.cos(b.getAngleRadians()),
        bY = b.getMagnitude() * Math.sin(b.getAngleRadians());

        //Add cartesian coordinates
        double mX = aX + bX, mY = aY + bY;

        //Convert to new vector with direction and magnitude
        return new SwerveVector(safeAtan2(mY, mX), Math.sqrt((mX * mX) + (mY * mY)));
    }
    /**
     * Add to angle of vector
     * @param radsToAdd - amount in radians to add
     */
    public void addRadsToAngle(double radsToAdd) {
        direction += radsToAdd;
        direction = direction % (Math.PI*2);
    }

    /**
     * Convert from 0 to 2PI to -PI to PI
     * @param angle - angle from 0 to 2pi
     * @return angle from -PI to PI
     */
    public static double convertToAltDomain(double angle) {
        return angle <= Math.PI ? angle : 2 * Math.PI - angle;
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
        direction = (angle / 57.29578) % (Math.PI*2);
    }

    /**
     * Sets the vector angle in degrees
     */
    public void setAngleRadians(double angle){
        direction = angle % (Math.PI*2);
    }

    /**
     * Returns the magnitude of the vector
     * @return vector magnitude
     */
    public double getMagnitude() {
        return power;
    }
}
