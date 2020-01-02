package frc.maths;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Provides some commonly used math functions.
 */
public class MathUtils {

    /**
     * Limits the input {@code i} to be between {@code -limit} and {@code +limit}. 
     * @param i Input to truncate.
     * @param limit Bounds of the input. Should be passed as a positive number.
     * @return A value between {@code -limit} and {@code +limit}.
     */
    public static double limit(double i, double limit) {
        return (Math.abs(i) < limit) ? i : limit * (i < 0 ? -1 : 1);
    }

    /**
     * Returns the value closest to zero.
     * @param a The first value.
     * @param b The second value.
     * @return Parameter closest to zero.
     */
    public static double absMin(double a, double b) {
        if(Math.abs(a) <= Math.abs(b))
            return a;
        else
            return b;
    }
    
    /**
     * Normalizes an angle in the range of [0, 2pi).
     * @param angle Angle to wrap in radians.
     * @return Angle between [0, 2pi).
     */
    public static double normalizeAngleRad(double angle) {
        double scaled = angle % (Math.PI * 2);
        if (scaled < 0)
            scaled += Math.PI * 2;
        return scaled;
    }

    /**
     * Normalizes an angle in the range of [-pi, pi).
     * @param angle Angle to wrap in radians.
     * @return Angle between [-pi, pi).
     */
    public static double normalizeAngleRad2(double angle) {
        double scaled = (angle + Math.PI) % (Math.PI * 2);
        if(scaled < 0)
            scaled += Math.PI * 2;
        return scaled - Math.PI;
    }

    /**
     * Normalizes an angle in the range of [0, 360).
     * @param angle Angle to wrap in degrees.
     * @return Angle between [0, 360).
     */
    public static double normalizeAngleDeg(double angle) {
        double scaled = angle % (360);
        if(scaled < 0)
            scaled += 360;
        return scaled;
    }

    /**
     * Normalizes an angle in the range of [-180, 180).
     * @param angle Angle to wrap in degrees.
     * @return Angle between [-180, 180).
     */
    public static double normalizeAngleDeg2(double angle) {
        double scaled = (angle + 180) % (360);
        if(scaled < 0)
            scaled += 360;
        return scaled - 180;
    }

    /**
     * Normalizes an amount of encoder ticks such that it is between 
     * {@code -ticksPerRev/2} and {@code ticksPerRev/2}.
     * @param ticks Encoder count to adjust in native ticks.
     * @param ticksPerRev Number of native encoder ticks per revolution.
     * @return Adjusted amount of ticks.
     */
    //TODO: Actually test this function. It probably works though?
    public static double normalizeAngleNative(double ticks, double ticksPerRev) {
        double scaled = (ticks + ticksPerRev / 2) % (ticksPerRev);
        if(scaled < 0)
            scaled += ticksPerRev;
        return scaled - ticksPerRev / 2;
    }

    /**
     * Checks if either Joystick axis (x and y) are outside of the bounds specified by the vector.
     * @param joystick Joystick to check.
     * @param band Deadband for this joystick.
     * @return {@code true} if any axis is outside the band. 
     */
    public static boolean outOfDeadband(Joystick joystick, Vector2d band) {
        if(Math.abs(joystick.getX()) > band.getX() || Math.abs(joystick.getY()) > band.getY())
            return true;
        else return false;
    }

    /**
     * Maps the joystick output so that it is zero if the axis is inside the deadband or
     * between {@code 0..1} if outisde of it. It also adjusts the zero point so that it
     * starts at the edge of the deadband.
     * Also rotates the resultant angle so that forward has an angle of zero.
     * @param joystick Joystick to read the axis (axes?) from.
     * @param band Deadband of the joystick, where it will assume the joystick is neutral.
     * @param invertY Whether to invert the y axis.
     * @param invertX Whether to invert the x axis.
     * @return Mapped axis values.
     * @see #adjustDeadband(Vector2d, Vector2d)
     */
    public static Vector2d adjustDeadband(Joystick joystick, Vector2d band, boolean invertY, boolean invertX) {
        int xMultipier  = invertX ? 1 : -1;
        int yMultiplier = invertY ? 1 : -1;

        return adjustDeadband(new Vector2d(joystick.getX() * xMultipier, joystick.getY() *yMultiplier).rotate(Math.PI/2), band);
    }

    /**
     * Maps the joystick output so that it is zero if the axis is inside the deadband or
     * between {@code 0..1} if outisde of it. It also adjusts the zero point so that it
     * starts at the edge of the deadband.
     * @param Input Input to retrieve values from.
     * @param band Deadband of the input, where it will assume the input is neutral.
     * @return Mapped axis values.
     */
    public static Vector2d adjustDeadband(Vector2d input, Vector2d band) {
        return new Vector2d(adjustBand(input.getX(), band.getX()), adjustBand(input.getY(), band.getY()));
    }

    /**
     * Helper function for checking the deadband.
     * @param input Input value.
     * @param min Minimum value for the input.
     * @return Adjusted value.
     */
    private static double adjustBand(double input, double min) {
        double abs = Math.abs(input);
        if(abs < min)
            return 0;
        double sign = Math.signum(input); // Gets the sign of the input.
        return sign * map(abs, min, 1, 0 ,1);
    }

    /**
     * Scales a value {@code input} in the range {@code a..b} to the range {@code c..d}, such that
     * f(a) = c and f(b) = d.
     * Also known as an affine transformation.
     * @param input Value to map.
     * @param a Initial low bound.
     * @param b Initial high bound.
     * @param c Final low bound.
     * @param d Final high bound.
     * @return Mapped value.
     */
    public static double map(double input, double a, double b, double c, double d) {
        return ((input - a)*(d-c)/(b-a))+c;
    }
}