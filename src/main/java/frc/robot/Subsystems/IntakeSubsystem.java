package frc.robot.Subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

    //singleton
    //static IntakeSubsystem instance = new IntakeSubsystem();
    //public static IntakeSubsystem getInstance() {return instance;}

    //CANSparkMax AngMotor;
    //DutyCycleEncoder  Encoder;
    //SparkPIDController m_pidController;
    //CANSparkMax WheelMotor;

    //RelativeEncoder m_alternateEncoder;
    //Ultrasonic DistanceSensor;
    //PIDController angController;

    //boolean hasNote;

    //public enum IntakeSpeed{In,Out,Amp,Speaker,Off}    
    //public enum IntakeAng{Amp,Speaker,Extended}

    //IntakeSpeed Speed;

    //public IntakeSubsystem() {

        //AngMotor = new CANSparkMax(Constants.Intake.AngMotorID, MotorType.kBrushless);
        //m_pidController = AngMotor.getPIDController();
        //m_pidController.setFeedbackDevice(AngMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192));
        //m_pidController.setP(Constants.Intake.ControllerKP);
        //m_pidController.setI(Constants.Intake.ControllerKI);
        //m_pidController.setD(Constants.Intake.ControllerKD);
        //m_pidController.setOutputRange(-0.5, 0.5);
        
        //WheelMotor = new CANSparkMax(Constants.Intake.ShootMotorID, MotorType.kBrushless);
        //DistanceSensor = new Ultrasonic(Constants.Intake.PingChannel,Constants.Intake.EchoChannel);
        
        //angController = new PIDController(Constants.Intake.ControllerKP, Constants.Intake.ControllerKI, Constants.Intake.ControllerKD);
        //angController.setTolerance(Constants.Intake.ControllerTolerance);
    //}

    //@Override
    //public void periodic() {
        
        //hasNote = DistanceSensor.getRangeMM() < Constants.Intake.DistanceSensorThreshold;
        //m_pidController.setReference(10 / 360, CANSparkMax.ControlType.kPosition);
    //}

    //public void setWheelMotorTarget(IntakeSpeed Speed) {
    //    switch (Speed) {
    //        case In:
    //            WheelMotor.set(Constants.Intake.IntakeSpeed);
    //            break;
    //        case Out:
    //            WheelMotor.set(-Constants.Intake.IntakeSpeed);
    //            break;
    //        case Amp:
    //            WheelMotor.set(Constants.Intake.AmpSpeed);
    //            break;
    //        case Speaker:
    //            WheelMotor.set(Constants.Intake.SpeakerSpeed);
    //            break;
    //        case Off:
    //            WheelMotor.set(0);
    //            break;
    //        default:
    //            WheelMotor.set(0);
    //            break;
    //        
    //    }    
    //}

    //public void setAngTarget(IntakeAng Ang) {
    //    switch (Ang) {
    //        case Amp:
    //            m_pidController.setReference(Constants.Intake.ampAng / 360, CANSparkMax.ControlType.kPosition);
    //            break;
    //        case Speaker:
    //            m_pidController.setReference(Constants.Intake.speakerAng / 360, CANSparkMax.ControlType.kPosition);
    //            break;
    //        case Extended:
    //            m_pidController.setReference(Constants.Intake.ExtendedAngle / 360, CANSparkMax.ControlType.kPosition);
    //            break;
    //        default:
    //            m_pidController.setReference(Constants.Intake.speakerAng / 360, CANSparkMax.ControlType.kPosition);
    //            break;
    //    }
    //}

   //public boolean getAtSetpoint(){
   //    return angController.atSetpoint();
   //}

    //public boolean getHasNote(){
    //    return hasNote;
    //}

    
}
