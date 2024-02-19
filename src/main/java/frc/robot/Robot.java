// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  XboxController driveController = new XboxController(0);
  //XboxController coDriverController = new XboxController(1);

  CANSparkMax testMotor = new CANSparkMax(15, MotorType.kBrushless);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    //DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(
    //    driveController.getLeftX() * Constants.attainableMaxTranslationalSpeedMPS, 
    //    driveController.getLeftY() * Constants.attainableMaxTranslationalSpeedMPS, 
    //    driveController.getRightX() * Constants.attainableMaxRotationalVelocityRPS)
    //);  

    //DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(
    //    0.5,
    //    0,
    //    0)
    //);  

    testMotor.set( driveController.getLeftX()/2 );
  
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
