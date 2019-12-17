package frc.maths;

/**
 * Contains swerve-specific math operations.
 */
public class SwerveUtils {

    /**
     * Normalizes a batch of vectors such that the longest vector is capped at the magnitude
     * of the limit, while the other vectors are rescaled proportionately.
     * Important for achieving desired motion when wheels would normally go over max velocity.
     * @param limit Magnitude limit of largest vector.
     * @param vectors List of vectors to scale proportionately to each other.
     * @return Array of scaled vectors.
     */
    public static Vector2d[] batchNormalize(double limit, Vector2d... vectors) {
        // Get the greatest magnitude from the batch of vectors.
        double maxMagnitude = 0;
        for(Vector2d vector : vectors) {
            if(vector.getMagnitude() > maxMagnitude)
                maxMagnitude = vector.getMagnitude();
        }

        // Check if the greatest magnitude is actually over the limit.
        if(limit >= maxMagnitude)
        return vectors;

        // Normalize the vectors to the limit.
        Vector2d[] normalized = new Vector2d[vectors.length];
        for(int i = 0; i < vectors.length; i++) {
            normalized[i] = vectors[i].scale(limit / maxMagnitude);
        }
        
        // Return the normalized vector array.
        return normalized;
    }

    /**
     * Works by calculating a "rotation vector", which is perpendicular to the swerve module's position vector
     * and has a magnitude proportional to the desired rotation speed. This vector is added to the translation
     * vector to calculate the drive vector.
     * In layman's terms, this accounts for the differential part of the differential swerve drive. Rotation vector handles the wheel turning, and translational
     * vector handles how fast the wheel spins.
     * @param gyroHeading Robot heading.
     * @param rotationValue "Power" of rotation (-1..1).
     * @param translationVector Field-centered translation using a joystick.
     * @param modulePosition Position vector of module relative to rotation center.
     * @return Vector representing wheel angle and speed.
     */
    public static Vector2d calculateDriveVector(double gyroHeading, double rotationValue, Vector2d translationVector, Vector2d modulePosition) {
        Vector2d transVecRot = translationVector.rotate(gyroHeading);
        Vector2d rotationVec = modulePosition.normalize(rotationValue).rotate(Math.PI/2);
        
        Vector2d driveVec = transVecRot.add(rotationVec);
        return driveVec;
    }

    /**
     * Calculates and normalizes several modules in one line.
     * @param gyroHeading Robot heading.
     * @param rotationValue "Power" of rotation (-1..1).
     * @param translationVector Field-centered translation using a joystick.
     * @param modulePos Position vector of module relative to rotation center.
     * @return Vector representing wheel angle and speed.
     */
    public static Vector2d[] calculateAllModules(double gyroHeading, double rotationValue, Vector2d translationVector, Vector2d... modulePos) {
        Vector2d[] driveVecs = new Vector2d[modulePos.length];
        for(int i = 0; i < modulePos.length; i++) {
            driveVecs[i] = calculateDriveVector(gyroHeading, rotationValue, translationVector, modulePos[i]);
        }

        Vector2d[] driveVecsNormalized = batchNormalize(1.0, driveVecs);
        return driveVecsNormalized;
    }
}