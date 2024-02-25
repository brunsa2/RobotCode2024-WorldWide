// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.DrivetrainSubsystem;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  XboxController driveController = new XboxController(0);
  XboxController coDriverController = new XboxController(1);

  //crap code//
  CANSparkMax AngMotor = new CANSparkMax(Constants.Intake.AngMotorID, MotorType.kBrushless);
  CANSparkMax WheelMotor = new CANSparkMax(Constants.Intake.ShootMotorID, MotorType.kBrushless);
  boolean shooting = false;
  CANSparkMax shooter1 = new CANSparkMax(Constants.Shooter.aID, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(Constants.Shooter.bID, MotorType.kBrushless);

  PneumaticsControlModule PCM = new PneumaticsControlModule();
  DoubleSolenoid solenoid;
  //crap code//

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    PCM.enableCompressorDigital();
    solenoid = PCM.makeDoubleSolenoid(1, 0);
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


    //TODO proper velocity controll/current controll, "shooting mode" stop all motors while shooting, max acc, drive auton, intake auton, genralize swerve code
    if(driveController.getXButton())
      {solenoid.set(Value.kReverse);}    
    if(driveController.getAButton())
      {solenoid.set(Value.kForward);}


    if(driveController.getLeftBumper()){AngMotor.set(0.5);}
    if(driveController.getRightBumper()){AngMotor.set(-0.5);}
    if(!driveController.getLeftBumper() && !driveController.getRightBumper()) {AngMotor.set(0);}

    if(driveController.getLeftTriggerAxis() > 0.1){WheelMotor.set(0.5);}else
    if(driveController.getRightTriggerAxis() > 0.1){WheelMotor.set(-0.5);}else 
    {WheelMotor.set(0);}
    
    if(driveController.getYButtonPressed()){shooting=!shooting;}
    
    shooter1.set(shooting?-1:0);
    shooter2.set(shooting?1:0);

    double x = driveController.getLeftX();
    double y = driveController.getLeftY();
    double rx = driveController.getRightX();

    if(Math.hypot(x,y) < Constants.controllerDeadband){ //FIXME deadband removed
        x = 0.0;
        y = 0.0;
    }

    if(Math.abs(rx) < Constants.controllerDeadband){
      rx = 0.0;
    } 

    DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(
        y * Constants.attainableMaxTranslationalSpeedMPS, 
        x * Constants.attainableMaxTranslationalSpeedMPS, 
        rx * Constants.attainableMaxRotationalVelocityRPS)
        );  

    
    //DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(
    //    0.5,
    //    0,
    //    0)
    //);  
  
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
