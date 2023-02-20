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

    public static final class Calculation{
        //Limelightの角度
        public static final double LimelightMountAngleDegrees = 34.5;
        //Limelightの高さ
        public static final double LimelightLensHeightCentis = 81.5;
        //ターゲットの高さ
        public static final double GoalHeightCentis = 166;
    }

    public static void ConstInit() {
        
    }
}
