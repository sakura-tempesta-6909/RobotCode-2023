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

    public static final class calculation {
        public static double obliqueLine = Math.pow(16, 2) + Math.pow(9, 2);
        public static double a = 68.5 * 16 / Math.sqrt(obliqueLine);
        public static double b = 68.5 * 9 / Math.sqrt(obliqueLine);


    }

    public static void ConstInit() {
        
    }
}
