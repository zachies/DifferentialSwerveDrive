/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.DiffSwerveModule;
import frc.subsystems.DiffSwerveModule.ModuleID;

public class Robot extends TimedRobot {

  DiffSwerveModule frontRight;

  @Override
  public void robotInit() {
    frontRight = new DiffSwerveModule(ModuleID.FR);
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
    SmartDashboard.putNumber("Motor2Pos", frontRight.getMotor2Pos());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    frontRight.process();
  }

}
