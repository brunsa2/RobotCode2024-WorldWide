package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class swerveModule extends SubsystemBase{

    //drive 
    CANSparkMax driveMotor;
    SparkAbsoluteEncoder driveMotorEncoder;
    SparkPIDController  driveController;
    //steer
    CANSparkMax steerMotor;
    SparkAbsoluteEncoder steerMotorEncoder;
    PIDController steerController;
    //module encoder 
    CANcoder moduleEncoder;
    double encoderOffsetRotations;
    //conversion factors
    final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    final double GEAR_RATIO = 1.0 / 6.75;
    final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;
    final double STEER_POSITION_CONVERSION = 1;
    final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.0;

    //CONSTRUCTOR//
        public swerveModule(int driveMotorID, int steerMotorID, int encoderID, Double encoderOffsetRotations){

            //drive motor 
            driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            driveMotor.restoreFactoryDefaults();
            driveMotor.setIdleMode(IdleMode.kBrake);
            driveMotor.setSmartCurrentLimit(40);
            driveMotor.burnFlash();
            //drive encoder
            driveMotorEncoder = driveMotor.getAbsoluteEncoder();
            //steer motor
            steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
            steerMotor.restoreFactoryDefaults();
            steerMotor.setIdleMode(IdleMode.kCoast);
            steerMotor.setSmartCurrentLimit(20);
            steerMotor.burnFlash();
            // module encoder
            moduleEncoder = new CANcoder(encoderID,"Default Name");
            this.encoderOffsetRotations = encoderOffsetRotations;

            //controllers
            //driveController = driveMotor.getPIDController();
            //driveController.setP(Constants.Modules.SpeedKP);
            //driveController.setI(Constants.Modules.SpeedKI);
            //driveController.setD(Constants.Modules.SpeedKD);

            steerController = new PIDController(Constants.Modules.SteerKP, Constants.Modules.SteerKI, Constants.Modules.SteerKD);
            steerController.enableContinuousInput(0, 1);

        }
    //DRIVE//
        public void setTargetState(SwerveModuleState targetState) {
            //PID experement
            //  steerMotor.set(-steerController.calculate(getModuleAngRotations(),targetState.angle.getRotations()));
            //  driveController.setReference(targetState.speedMetersPerSecond / DRIVE_VELOCITY_CONVERSION, ControlType.kVelocity);

            // FUNCTIONING
            steerMotor.set(-steerController.calculate(getModuleAngRotations(),targetState.angle.getRotations()));
            driveMotor.set(targetState.speedMetersPerSecond/Constants.attainableMaxModuleSpeedMPS); 
        }
    //FEEDBACK//
        public double getModuleAngRotations(){
            return moduleEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffsetRotations;
        }
        
        public SwerveModulePosition getModulePosition() {
            return new SwerveModulePosition(
                driveMotorEncoder.getPosition(), //FIXME i broke this sorry
                Rotation2d.fromRotations(getModuleAngRotations())
            );  
        }

        public SwerveModuleState getSwerveModuleState() {
            return new SwerveModuleState(
                driveMotorEncoder.getVelocity(), //FIXME i broke this sorry
                Rotation2d.fromRotations(getModuleAngRotations()));
        }
    ////
}