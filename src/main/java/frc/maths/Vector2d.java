package frc.maths;

/**
 * Two-dimensional vector implementation.
 */
//TODO: Implement more polar helpers!
public class Vector2d {
    private final double x;
    private final double y;

    /**
     * Creates a new instance of a vector with the supplied x- and y-coordinates.
     * @param x X-coordinate.
     * @param y Y-coordinate.
     */
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return Y value of the vector.
     */
    public double getY() {
        return y;
    }

    /**
     * @return X value of the vector.
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the magnitude (length) of the vector.
     * @return Length of the vector.
     */
    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /**
     * Gets the angle of the vector.
     * @return Angle of vector in radians.
     */
    public double getAngle() {
        return MathUtils.normalizeAngleRad(Math.atan2(y, x));
    }

    /**
     * Adds two vectors together.
     * Maybe one day Java will support operator overloading.
     * @param addend Vector to add.
     * @return Sum of two vectors.
     */
    public Vector2d add(Vector2d addend) {
        return new Vector2d(x + addend.getX(), y + addend.getY());
    }

    /**
     * Scales a vector by the specified amount.
     * Angle will remain the same, but the magnitude will change.
     * @param scale Scale factor, such as 2.
     * @return Scaled vector.
     */
    public Vector2d scale(double scale) {
        return new Vector2d(x * scale, y * scale);
    }

    /**
     * Normalizes the vector to a unit length.
     * Angle will remain the same, but the magnitude will change.
     * @return Normalized vector.
     */
    public Vector2d normalize() {
        return scale(1.0 / getMagnitude());
    }

    /**
     * Normalizes the vector to a target length.
     * Angle will remain the same, but the magnitude will change.
     * @param target Target length.
     * @return Normalized vector.
     */
    public Vector2d normalize(double target) {
        return scale(target / getMagnitude());
    }

    /**
     * Rotates the vector by the specified {@code angle} in radians.
     * @param angle Amount to rotate vector by.
     * @return Rotated vector.
     */
    public Vector2d rotate(double angle) {
        return new Vector2d(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle));
    }

    /**
     * Computes the dot product of two vectors.
     * @param other Other Vector2d to compute dot product with.
     * @return Dot product.
     */
    public double dotProduct(Vector2d other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    /**
     * Returns the vector reflected into the first quadrant.
     * @return Vector in first quadrant.
     */
    public Vector2d abs() {
        return new Vector2d(Math.abs(x), Math.abs(y));
    }

    /**
     * Clones the current vector.
     */
    public Vector2d clone() {
        return new Vector2d(x, y);
    }

    @Override
    public String toString() {
        return String.format("(%s, %s)", x, y);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector2d)) {
            return false;
        }
        Vector2d other = (Vector2d) obj;
        if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x)) {
            return false;
        }
        if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y)) {
            return false;
        }
        return true;
    }
}