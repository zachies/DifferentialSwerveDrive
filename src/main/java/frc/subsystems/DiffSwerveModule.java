package frc.subsystems;

import frc.maths.MathUtils;
import frc.maths.Vector2d;
import static frc.robot.RobotMap.*;

/**
 * DiffSwerveModule
 */
public class DiffSwerveModule {

    public enum ModuleID {
        FR, FL, BR, BL;
    }

    private Vector2d positionVec;

    private SparkMax motor1;
    private SparkMax motor2;

    private ModuleID moduleID;

    public static final int kSteeringCountsPerRev = 5;

    public DiffSwerveModule(ModuleID id) {
        moduleID = id;
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

        // Sets the scale factor of the encoders.

    }

    public void process() {
        motor1.process();
        motor2.process();
    }

    public Vector2d getPosVec() {
        return positionVec;
    }

    public void setSpeed(double motor1Speed, double motor2Speed) {
        motor1.set(motor1Speed);
        motor2.set(motor2Speed);
    }

    public double getMotor1Pos() {
        return motor1.getEncPosition();
    }

    /**
     * Supposedly returns the sum of the encoders on both motors, which is the angular position of the module.
     * Neat!
     * @return
     */
    public double getMotor2Pos() {
        return (motor1.getEncPosition() + motor2.getEncPosition()) / 2;
    }

    /**
     * TODO: Check if this is actually how the velocity is calculated.
     */
    public double getModuleVel() {
        return (motor1.getEncVelocity() + motor2.getEncVelocity()) / 2;
    }

    public int getModulePositionTrunc() {
        return MathUtils.normalizeAngleNative((int)getMotor2Pos(), kSteeringCountsPerRev);
    }

    public double getModulePositionRad() {
        return MathUtils.normalizeAngleRad2((double)(getModulePositionTrunc())/(double)kSteeringCountsPerRev * Math.PI * 2.0);
    }

    public void stop() {
        motor1.set(0);
        motor2.set(0);
    }

    //TODO: can you zero the encoders? Is it even necessary?

    public void setPositionAndSpeedNative(double drive, int target) {
        int diff = MathUtils.normalizeAngleNative(target - (int)getMotor2Pos(), kSteeringCountsPerRev);
        double setpoint = getMotor2Pos() + diff;
    }
}