package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private ModuleID moduleID;

    public DiffSwerveModule(ModuleID id) {
        moduleID = id;
        switch(id) {
            case FR:
                positionVec = new Vector2d(kROBOT_WIDTH/2.0, kROBOT_LENGTH/2.0);
                motor1 = new CANSparkMax(kFR_MOTOR_1, MotorType.kBrushless);
                motor2 = new CANSparkMax(kFR_MOTOR_2, MotorType.kBrushless);
                break;
            case FL:
                positionVec = new Vector2d(-kROBOT_WIDTH/2.0, kROBOT_LENGTH/2.0);
                motor1 = new CANSparkMax(kFL_MOTOR_1, MotorType.kBrushless);
                motor2 = new CANSparkMax(kFL_MOTOR_2, MotorType.kBrushless);
                break;
            case BR:
                positionVec = new Vector2d(kROBOT_WIDTH/2.0, -kROBOT_LENGTH/2.0);
                motor1 = new CANSparkMax(kBR_MOTOR_1, MotorType.kBrushless);
                motor2 = new CANSparkMax(kBR_MOTOR_2, MotorType.kBrushless);
                break;
            case BL:
                positionVec = new Vector2d(-kROBOT_WIDTH/2.0, -kROBOT_LENGTH/2.0);
                motor1 = new CANSparkMax(kBL_MOTOR_1, MotorType.kBrushless);
                motor2 = new CANSparkMax(kBL_MOTOR_2, MotorType.kBrushless);
                break;
            default:
                System.out.println("ERROR: Swerve module not found!");
                break;
        }

        // Polarities, directions, and phases
        motor1.setInverted(false);
        motor2.setInverted(false);
        motor1.getEncoder().setInverted(false);
        motor2.getEncoder().setInverted(false);
    }
}