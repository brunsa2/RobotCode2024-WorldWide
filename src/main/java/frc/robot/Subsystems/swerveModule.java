package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class swerveModule extends SubsystemBase{

    CANSparkMax driveMotor;
    SparkAbsoluteEncoder driveMotorEncoder;
    PIDController driveController;
    CANSparkMax steerMotor;
    SparkAbsoluteEncoder steerMotorEncoder;
    PIDController steerController;
    CANcoder moduleEncoder;
    double encoderOffsetRotations;

    double WHEEL_DIAMETER = Units.inchesToMeters(3);
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    double GEAR_RATIO = 1.0 / 6.75;
    double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;

    double STEER_POSITION_CONVERSION = 1;
    double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.0;

    public swerveModule(int driveMotorID, int steerMotorID, int encoderID, Double encoderOffsetdegrese){
        
        //drive motor 
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        //drive encoder
        driveMotorEncoder = driveMotor.getAbsoluteEncoder();
        driveMotorEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
        driveMotorEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
        //drive controller
        driveController = new PIDController(Constants.Modules.SpeedKP, Constants.Modules.SpeedKI, Constants.Modules.SpeedKD);
        
        //steer motor
        steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless); 
        //drive controller
        steerController = new PIDController(Constants.Modules.SteerKP, Constants.Modules.SteerKI, Constants.Modules.SteerKD);
        steerController.enableContinuousInput(0, 1);
        
        // module encoder
        moduleEncoder = new CANcoder(encoderID);
        encoderOffsetRotations = encoderOffsetdegrese / 360;
        
    }

    public void setTargetState(SwerveModuleState targetState) {
        steerController.setSetpoint(targetState.angle.getRotations());
        driveController.setSetpoint(targetState.speedMetersPerSecond);
    }

    @Override
    public void periodic(){
        steerMotor.set(steerController.calculate(getModuleAngRotations()));
        driveMotor.set(driveController.calculate(driveMotorEncoder.getVelocity()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveMotorEncoder.getPosition(), 
            Rotation2d.fromRotations(getModuleAngRotations())
        );  
    }

    public double getModuleAngRotations(){
        return moduleEncoder.getAbsolutePosition().getValue() - encoderOffsetRotations;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            driveMotorEncoder.getVelocity(), 
            Rotation2d.fromRotations(steerMotorEncoder.getPosition()-encoderOffsetRotations));
    }
    
}