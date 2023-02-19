package frc.robot.subClass;

public class Const {
    public static final class Ports {
        public static final int DriveController = 0;
        public static final int OperateController = 1;

        public static final int DriveRightFront = 0;
        public static final int DriveLeftFront = 1;
        public static final int DriveRightBack = 2;
        public static final int DriveLeftBack = 3;
    }

    public static final class Speeds {
        public static double Neutral = 0;

        public static double FastDrive = 0.8;
        public static double MidDrive = 0.5;
        public static double SlowDrive = 0.3;

    }

    public static final class Arms {
        public static double FirstArmLength = 90.0;
        public static double SecondArmLength = 90.0;
        public static double FirstArmBarycenter = 45.0;
        public static double SecondArmBarycenter = 45.0;
        public static double FirstArmMass = 0.0;
        public static double SecondArmMass = 0.0;
        public static double TargetModifyRatio = 1;
        public static double TakeUpLengthAfterGrab = 20.0;
        public static double TargetPositionLimit = Math.sqrt(Math.pow(FirstArmLength, 2) + Math.pow(SecondArmLength, 2));
        public static double kP1 = 0.2;
        public static double kI1 = 0.2;
        public static double kD1 = 0.01;

        public static double kP2 = 0.1;
        public static double kI2 = 0.000;
        public static double kD2 = 0.000;

        public static double Encoder1CountPerRotation = 14.3469;
        public static double Encoder2CountPerRotation = 9.196;
    }

    public static void ConstInit() {
        
    }
}
