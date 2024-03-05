// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.DrivetrainSubsystem;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;                                  

  XboxController driveController = new XboxController(0);
  XboxController coDriverController = new XboxController(1);

  //crap code//
  CANSparkMax AngMotor = new CANSparkMax(Constants.Intake.AngMotorID, MotorType.kBrushless);
  CANSparkMax WheelMotor = new CANSparkMax(Constants.Intake.ShootMotorID, MotorType.kBrushless);
  boolean shooting = false;
  boolean intaking = false;
  CANSparkMax shooter1 = new CANSparkMax(Constants.Shooter.aID, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(Constants.Shooter.bID, MotorType.kBrushless);
  public void setShooterSpeed(double speed){shooter1.set(-speed);shooter2.set(speed);}

  PneumaticsControlModule PCM = new PneumaticsControlModule();
  DoubleSolenoid solenoid;
  
  public Command shoot = Commands.runOnce(()->{setShooterSpeed(1.0);}).andThen(Commands.waitSeconds(1)).andThen(Commands.runOnce(()->{WheelMotor.set(1);})).andThen(Commands.waitSeconds(1)).andThen(Commands.runOnce(()->{WheelMotor.set(0);setShooterSpeed(0);}));
  public Command taxi = Commands.runOnce(()->{DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(-2,0,0));}).andThen(Commands.waitSeconds(4)).andThen(()->{DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds());});
  //crap code//

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    //camera//
    //CameraServer.startAutomaticCapture();

    PCM.enableCompressorDigital();
    solenoid = PCM.makeDoubleSolenoid(1, 0);
    solenoid.set(Value.kReverse);

    AngMotor.restoreFactoryDefaults();
    AngMotor.setIdleMode(IdleMode.kBrake);
    AngMotor.setSmartCurrentLimit(40);
    AngMotor.burnFlash();
    WheelMotor.restoreFactoryDefaults();
    WheelMotor.setIdleMode(IdleMode.kCoast);
    WheelMotor.setSmartCurrentLimit(40);
    WheelMotor.burnFlash();
    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(40);
    shooter1.burnFlash();
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(40);
    shooter2.burnFlash();
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
    m_autonomousCommand = taxi;
    //m_robotContainer.getAutonomousCommand();

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
    
    //climber
    if(driveController.getXButtonPressed()){solenoid.set(Value.kReverse);}    
    if(driveController.getAButtonPressed()){solenoid.set(Value.kForward);}
    //intake
    AngMotor.set(coDriverController.getLeftBumper()?0.5:coDriverController.getRightBumper()?-0.5:0);
    WheelMotor.set(coDriverController.getLeftTriggerAxis() > 0.1?0.5:coDriverController.getRightTriggerAxis() > 0.1?-0.5:0);
    WheelMotor.set((coDriverController.getLeftTriggerAxis() * 0.3)+(-coDriverController.getRightTriggerAxis()));

    //shooter
    if(coDriverController.getYButtonPressed()){shooting=!shooting;intaking=false;}    
    if(coDriverController.getBButtonPressed()){intaking=!intaking;shooting=false;}
    setShooterSpeed(shooting?1:intaking?-0.1 :0.3);
    coDriverController.setRumble(RumbleType.kBothRumble, shooting?1:0);
    //coDriverController.setRumble(RumbleType.kRightRumble, intaking?0.5:1);

    //drive
    if(driveController.getStartButtonPressed()){
      DrivetrainSubsystem.getInstance().resetGyro();
    }


    double x = driveController.getLeftX(),y = driveController.getLeftY(),theta = driveController.getRightX();
    if(Math.hypot(x,y) < Constants.controllerDeadband){x = 0.0; y = 0.0;}
    if(Math.abs(theta) < Constants.controllerDeadband){theta = 0.0;} 

    DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(
        y * Constants.attainableMaxTranslationalSpeedMPS, 
        x * Constants.attainableMaxTranslationalSpeedMPS, 
        theta * Constants.attainableMaxRotationalVelocityRPS)
    );
  
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
