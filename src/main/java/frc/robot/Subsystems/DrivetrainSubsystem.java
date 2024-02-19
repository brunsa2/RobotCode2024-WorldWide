package frc.robot.Subsystems;


//import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase{

    //singleton
    static DrivetrainSubsystem instance = new DrivetrainSubsystem();
    public static DrivetrainSubsystem getInstance() {return instance;}

    //ADIS16470_IMU IMU;
    //Pigeon2 pigeon;
    swerveModule[] modules;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    ChassisSpeeds m_chassisSpeeds;

    public DrivetrainSubsystem() {

    //IMU = new ADIS16470_IMU();
    //pigeon = new Pigeon2(Constants.PigeonID);
    modules[0] = new swerveModule(Constants.Modules.FrontRightDriveID,Constants.Modules.FrontRightSteerID,Constants.Modules.FrontRightEncoderID,Constants.Modules.FrontRightEncoderOffset);
    modules[1] = new swerveModule(Constants.Modules.FrontLeftDriveID, Constants.Modules.FrontLeftSteerID, Constants.Modules.FrontLeftEncoderID, Constants.Modules.FrontLeftEncoderOffset ); 
    modules[2] = new swerveModule(Constants.Modules.RearLeftDriveID,  Constants.Modules.RearLeftSteerID,  Constants.Modules.RearLeftEncoderID,  Constants.Modules.RearLeftEncoderOffset  );
    modules[3] = new swerveModule(Constants.Modules.RearRightDriveID, Constants.Modules.RearRightSteerID, Constants.Modules.RearRightEncoderID, Constants.Modules.RearRightEncoderOffset );
    kinematics = new SwerveDriveKinematics(Constants.moduleLocations);

    }

    //public Rotation2d getGyroscopeRotation() {
    //    return Rotation2d.fromDegrees(IMU.getAngle());
    //}

    //public Rotation2d getGyroscopeRotation() {
    //    return Rotation2d.fromDegrees(pigeon.getRoll());
    //}

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {modules[0].getModulePosition(),modules[1].getModulePosition(),modules[2].getModulePosition(),modules[3].getModulePosition()};
    }

    public void drive(ChassisSpeeds  chassisSpeeds){
        m_chassisSpeeds = chassisSpeeds;
    }

    //public void driveFieldRelative(ChassisSpeeds  chassisSpeeds){
    //    m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation());
    //}

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_chassisSpeeds, Constants.attainableMaxModuleSpeedMPS, Constants.attainableMaxTranslationalSpeedMPS, Constants.attainableMaxRotationalVelocityRPS);
        modules[0].setTargetState(desiredStates[0]);//TODO optimize desired states 
        modules[1].setTargetState(desiredStates[1]);
        modules[2].setTargetState(desiredStates[2]);
        modules[3].setTargetState(desiredStates[3]);
        
    }

    @Override
    public void periodic() {
        setModuleStates(kinematics.toSwerveModuleStates(m_chassisSpeeds));
    }


    public ChassisSpeeds getChasisSpeed(){
        return kinematics.toChassisSpeeds(
            modules[0].getSwerveModuleState(),
            modules[1].getSwerveModuleState(),
            modules[2].getSwerveModuleState(),
            modules[3].getSwerveModuleState()
        );
    }
 
}