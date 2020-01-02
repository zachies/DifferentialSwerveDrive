package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes telemetry about a Spark motor controller.
 */
public class SparkMax extends CANSparkMax {
    private String name;

    public CANPIDController pidController;
    public CANEncoder encoder;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public SparkMax(String name, int id) {
        super(id, MotorType.kBrushless);
        this.name = name;

        pidController = super.getPIDController();
        encoder = super.getEncoder();

        // PID Coefficients
        kP = 5e-5;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        /**
         * Smart Motion coefficients are set on a CANPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid
         * controller in Smart Motion mode - setSmartMotionMinOutputVelocity() will put
         * a lower bound in RPM of the pid controller in Smart Motion mode -
         * setSmartMotionMaxAccel() will limit the acceleration in RPM^2 of the pid
         * controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError() will
         * set the max allowed error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber(name + " P Gain", kP);
        SmartDashboard.putNumber(name + " I Gain", kI);
        SmartDashboard.putNumber(name + " D Gain", kD);
        SmartDashboard.putNumber(name + " I Zone", kIz);
        SmartDashboard.putNumber(name + " Feed Forward", kFF);
        SmartDashboard.putNumber(name + " Max Output", kMaxOutput);
        SmartDashboard.putNumber(name + " Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber(name + " Max Velocity", maxVel);
        SmartDashboard.putNumber(name + " Min Velocity", minVel);
        SmartDashboard.putNumber(name + " Max Acceleration", maxAcc);
        SmartDashboard.putNumber(name + " Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber(name + " Set Position", 0);
        SmartDashboard.putNumber(name + " Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean(name + " Mode", true);
    }

    /**
     * Updates the motor.
     */
    public void process() {
        double p = SmartDashboard.getNumber(name + " P Gain", 0);
        double i = SmartDashboard.getNumber(name + " I Gain", 0);
        double d = SmartDashboard.getNumber(name + " D Gain", 0);
        double iz = SmartDashboard.getNumber(name + " I Zone", 0);
        double ff = SmartDashboard.getNumber(name + " Feed Forward", 0);
        double max = SmartDashboard.getNumber(name + " Max Output", 0);
        double min = SmartDashboard.getNumber(name + " Min Output", 0);
        double maxV = SmartDashboard.getNumber(name + " Max Velocity", 0);
        double minV = SmartDashboard.getNumber(name + " Min Velocity", 0);
        double maxA = SmartDashboard.getNumber(name + " Max Acceleration", 0);
        double allE = SmartDashboard.getNumber(name + " Allowed Closed Loop Error", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
        if ((maxV != maxVel)) {
            pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }
        if ((allE != allowedErr)) {
            pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }

        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean(name + " Mode", false);
        if (mode) {
            setPoint = SmartDashboard.getNumber(name + " Set Velocity", 0);
            pidController.setReference(setPoint, ControlType.kVelocity);
            processVariable = encoder.getVelocity();
        } else {
            setPoint = SmartDashboard.getNumber(name + " Set Position", 0);
            /**
             * As with other PID modes, Smart Motion is set by calling the setReference
             * method on an existing pid object and setting the control type to kSmartMotion
             */
            pidController.setReference(setPoint, ControlType.kSmartMotion);
            processVariable = encoder.getPosition();
        }

        SmartDashboard.putNumber(name + " SetPoint", setPoint);
        SmartDashboard.putNumber(name + " Process Variable", processVariable);
        SmartDashboard.putNumber(name + " Output", getAppliedOutput());
    }

    public double getEncPosition() {
        return encoder.getPosition();
    }

    public double getEncVelocity() {
        return encoder.getVelocity();
    }
}