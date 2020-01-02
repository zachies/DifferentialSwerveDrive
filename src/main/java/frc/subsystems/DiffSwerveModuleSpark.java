package frc.subsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.maths.MathUtils;
import frc.maths.MiniPID;
import frc.maths.Vector2d;
import frc.subsystems.DiffSwerveModule.ModuleID;
import static frc.robot.RobotMap.*;

/**
 * DiffSwerveModuleSpark
 */
//TODO: figure out velocity of the motor.
public class DiffSwerveModuleSpark {
    private SparkMax motor1;
    private SparkMax motor2;
    private Vector2d positionVec;
    private MiniPID miniPID;

    // Number of encoder counts per revolution of the module.
    public static final int kModuleCountsPerRev = 5;

    public DiffSwerveModuleSpark(ModuleID id) {
        switch(id) {
            case FR:
                positionVec = new Vector2d(kROBOT_WIDTH/2.0, kROBOT_LENGTH/2.0);
                motor1 = new SparkMax("FR_MOTOR_1", kFR_MOTOR_1);
                motor2 = new SparkMax("FR_MOTOR_2", kFR_MOTOR_2);
                break;
            case FL:
                positionVec = new Vector2d(-kROBOT_WIDTH/2.0, kROBOT_LENGTH/2.0);
                motor1 = new SparkMax("FL_MOTOR_1", kFL_MOTOR_1);
                motor2 = new SparkMax("FL_MOTOR_2", kFL_MOTOR_2);
                break;
            case BR:
                positionVec = new Vector2d(kROBOT_WIDTH/2.0, -kROBOT_LENGTH/2.0);
                motor1 = new SparkMax("BR_MOTOR_1", kBR_MOTOR_1);
                motor2 = new SparkMax("BR_MOTOR_2", kBR_MOTOR_2);
                break;
            case BL:
                positionVec = new Vector2d(-kROBOT_WIDTH/2.0, -kROBOT_LENGTH/2.0);
                motor1 = new SparkMax("BL_MOTOR_1", kBL_MOTOR_1);
                motor2 = new SparkMax("BL_MOTOR_2", kBL_MOTOR_2);
                break;
            default:
                System.out.println("ERROR: Swerve module not found!");
                break;
        }

        // Set inverted status. Probably won't change any of these.
        motor1.setInverted(false);
        motor2.setInverted(false);

        miniPID = new MiniPID(0, 0, 0);
    }

    /**
     * Gets the angular position of the swerve module in native encoder ticks.
     * This measurement is not normalized; see @see getModulePosNativeNormalized() for
     * the normalized version in radians.
     * @return Module position in encoder ticks.
     */
    public double getModulePosNative() {
        return (motor1.getEncPosition() + motor2.getEncPosition()) / 2;
    }

    /**
     * Gets the angular position of the swerve module in native encoder ticks.
     * This measurement is normalized, meaning it is wrapped between
     * {@code -kModuleCountsPerRev/2} and {@code kModuleCountsPerRev/2}.
     * @return Normalized angular position of the swerve module.
     */
    public double getModulePosNativeNormalized() {
        return MathUtils.normalizeAngleNative(getModulePosNative(), kModuleCountsPerRev);
    }

    /**
     * Gets the angular position of the swerve module in radians.
     * This measurement is normalized, meaning it is wrapped between
     * {@code -pi} and {@code pi}.
     * @return Normalized angular position of the swerve module.
     */
    public double getModulePosRadNormalized() {
        return MathUtils.normalizeAngleRad2(getModulePosNative() * 2 * Math.PI / kModuleCountsPerRev);
    }

    /**
     * Sets the angle of the module and the speed of the motors.
     * NOTE: Speed is measured in encoder ticks!
     * NOTE: Angle is measured in radians!
     * @param vector
     */
    //TODO: make this a fully featured PID controller.
    public void set(Vector2d vector) {
        double speed = vector.getMagnitude();
        double angleSetpoint = vector.getAngle() * kModuleCountsPerRev / (2 * Math.PI);

        motor1.pidController.setReference(speed, ControlType.kVelocity);

        double kP = SmartDashboard.getNumber("kP", 0);
        double kI = SmartDashboard.getNumber("kI", 0);
        double kD = SmartDashboard.getNumber("kD", 0);
        
        miniPID.setPID(kP, kI, kD);
        double output = miniPID.getOutput(getModulePosNative(), angleSetpoint);

        // Make sure our output isn't TOO extreme.
        double boundedOutput = MathUtils.limit(output, 100);

        // Set the second motor to have speed - output.
        motor2.pidController.setReference(-speed + boundedOutput, ControlType.kVelocity);
    }

    /**
     * Angle is in radians!
     */
    public void set(double angle, double magnitude) {
        double speed = magnitude;
        double angleSetpoint = angle * kModuleCountsPerRev / (2 * Math.PI);

        motor1.pidController.setReference(speed, ControlType.kVelocity);

        double kP = SmartDashboard.getNumber("kP", 0);
        double kI = SmartDashboard.getNumber("kI", 0);
        double kD = SmartDashboard.getNumber("kD", 0);
        
        miniPID.setPID(kP, kI, kD);
        double output = miniPID.getOutput(getModulePosNative(), angleSetpoint);

        // Make sure our output isn't TOO extreme.
        double boundedOutput = MathUtils.limit(output, 100);

        // Set the second motor to have speed - output.
        motor2.pidController.setReference(-speed + boundedOutput, ControlType.kVelocity);
    }

    public double getM1Velocity() {
        return motor1.encoder.getVelocity();
    }

    public void printTelemetry() {
        SmartDashboard.putNumber("Module Position Native", getModulePosNative());
    }
}