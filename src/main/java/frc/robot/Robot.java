/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.maths.MathUtils;
import frc.maths.Vector2d;
import frc.subsystems.DiffSwerveModuleSpark;
import frc.subsystems.DiffSwerveModule.ModuleID;

public class Robot extends TimedRobot {

  //DiffSwerveModule frontRight;
  DiffSwerveModuleSpark frontRight;

  Joystick joystick = new Joystick(1);

  @Override
  public void robotInit() {
    frontRight = new DiffSwerveModuleSpark(ModuleID.FR);
  }

  @Override
  public void robotPeriodic() {
    frontRight.printTelemetry();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    Vector2d output = MathUtils.adjustDeadband(joystick, new Vector2d(0.15, 0.15), true, false);

    SmartDashboard.putNumber("Joystick Angle", output.getAngle());
    SmartDashboard.putNumber("Joystick Magnitude", output.getMagnitude());

    // Parameter d is the max velocity.
    double outputVelocity = MathUtils.map(output.getMagnitude(), 0, Math.sqrt(2), 0, 2000);
    SmartDashboard.putNumber("Output Velocity", outputVelocity);

    // Wraps the angle to [-pi, pi].
    double outputAngle = MathUtils.normalizeAngleRad2(output.getAngle());

    frontRight.set(outputAngle, outputVelocity);
  }

  @Override
  public void testInit() {
    //SmartDashboard.putNumber("arbFeedForward", 0);
    SmartDashboard.putNumber("Angle", 0);
    SmartDashboard.putNumber("Magnitude", 0);

    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);
  }

  @Override
  public void testPeriodic() {
    double angle = SmartDashboard.getNumber("Angle", 0);
    double magnitude = SmartDashboard.getNumber("Magnitude", 0);
    
    double x = magnitude * Math.cos(angle);
    double y = magnitude * Math.sin(angle);

    Vector2d posVec = new Vector2d(x, y);
    frontRight.set(posVec);

    //frontRight.process();
    //frontRight.setReference();
    //frontRight.updateTelemetry();
    //SmartDashboard.putNumber("Motor2Pos", frontRight.getMotor2Pos());
    //SmartDashboard.putNumber("FR Module Rotation", frontRight.getModulePosNative());
    SmartDashboard.putNumber("FR Module Velocity", frontRight.getM1Velocity());
  }

}
