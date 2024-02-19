package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    
    public static final Translation2d[] moduleLocations = {
        new Translation2d(0.29845,0.29845),//front right
        new Translation2d(-0.29845,0.29845), //front left
        new Translation2d(-0.29845,-0.29845),  //rear left
        new Translation2d(0.29845,-0.29845)  //rear right
    };
    
    //velocity constranints for swerve desaturate
    public static final double DriveBaseRadius = 0.42207203769;
    public static final double attainableMaxModuleSpeedMPS = 4.5;
    public static final double attainableMaxTranslationalSpeedMPS = 4.5;
    public static final double attainableMaxRotationalVelocityRPS = DriveBaseRadius * attainableMaxModuleSpeedMPS;

    public static final int PigeonID = 0;   

    public static double controllerDeadband = 0.1; 

    public interface Modules{
            public static final double SpeedKP = 0.02, SpeedKI = 0, SpeedKD = 0;
            public static final double SteerKP = 0.02, SteerKI = 0, SteerKD = 0;
        
            public static final int FrontLeftDriveID   = 4, FrontLeftSteerID   = 5, FrontLeftEncoderID = 6;
			public static final double FrontLeftEncoderOffset = 92.021;

            public static final int FrontRightDriveID   = 1, FrontRightSteerID   = 2, FrontRightEncoderID = 3;
            public static final double FrontRightEncoderOffset = 89.912;

            public static final int RearLeftDriveID   = 7, RearLeftSteerID   = 8, RearLeftEncoderID = 9;
            public static final double RearLeftEncoderOffset = 179.121;

            public static final int RearRightDriveID   = 10, RearRightSteerID   = 11, RearRightEncoderID = 12;
            public static final double RearRightEncoderOffset = 89.912;
        
    }

    public interface Intake{

        public static final double MaxSpeed = 0.30;//percent. positive is in negative is out
        public static final int AngMotorID = 16, ShootMotorID = 15, PingChannel = 0, EchoChannel = 1;

        public static final double ControllerTolerance = 1;//degrees 
        public static final double ControllerKP = 0.02, ControllerKI = 0, ControllerKD = 0;

        public static final double DistanceSensorThreshold = 0;

        public static final double ampAng = 0;
        public static final double speakerAng = 0;
        public static final double ExtendedAngle = 0;
        public static final double EncoderOffset = 0;

    }

    public interface Shooter{

        public static final int aID = 13,bID = 14;
		public static final double MaxSpeed = 1.00;//percent
        
    }

    public interface Climber{

        public static final double minPressure = 60, maxPressure = 120;

    }
    public interface Motion {
            public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
            public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
    }

}
