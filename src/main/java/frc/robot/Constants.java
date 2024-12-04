package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    
    public static final Translation2d[] moduleLocations = {
        new Translation2d(-0.29845,0.29845),  //front right ++
        new Translation2d(-0.29845,-0.29845), //front left  -+
        new Translation2d(0.29845,-0.29845),  //rear left   --
        new Translation2d(0.29845,0.29845)  //rear right  +-
    };
    
    //velocity constranints for swerve desaturate
    public static final double DriveBaseRadius = 0.42207203769;
    public static final double attainableMaxModuleSpeedMPS = 4.572;
    public static final double attainableMaxTranslationalSpeedMPS = attainableMaxModuleSpeedMPS;
    public static final double attainableMaxRotationalVelocityRPS = attainableMaxModuleSpeedMPS/DriveBaseRadius;

    public static final int PigeonID = 0;   

    public static double controllerDeadband = 0.15; 

    public interface Modules{
            public static final double SpeedKP = 0.001, SpeedKI = 0, SpeedKD = 0.0005;
            public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;
        
            public static final int FrontLeftDriveID   = 4, FrontLeftSteerID   = 5, FrontLeftEncoderID = 6;
			public static final double FrontLeftEncoderOffset = 0.08;//-0.423340 rotations raw = 0.000000 rotations

            public static final int FrontRightDriveID   = 1, FrontRightSteerID   = 2, FrontRightEncoderID = 3;
            public static final double FrontRightEncoderOffset = 0.99;//0.484131 rotations raw = -0.000244 rotations

            public static final int RearLeftDriveID   = 7, RearLeftSteerID   = 8, RearLeftEncoderID = 9;
            public static final double RearLeftEncoderOffset = 0.8;//0.283691 rotations raw = -0.000244 rotations

            public static final int RearRightDriveID   = 10, RearRightSteerID   = 11, RearRightEncoderID = 12;
            public static final double RearRightEncoderOffset = 0.95;//0.448730 rotations raw = 0.000244 rotations
        
    }

    public interface Motion {
            public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
            public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
    }

}


