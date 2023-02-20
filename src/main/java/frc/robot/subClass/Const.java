package frc.robot.subClass;
/** 常に決まっている数値(定数)をまとめたファイル */
public class Const {
    public static final class Ports {
        public static final int DriveController = 0;
        public static final int OperateController = 1;

        public static final int DriveRightFront = 0;
        public static final int DriveLeftFront = 1;
        public static final int DriveRightBack = 2;
        public static final int DriveLeftBack = 3;

        public static final int HandSolenoid = 0;

        public static final int IntakeSolenoid = 0;
    }

    public static final class Speeds {
        public static double Neutral = 0;

        public static double FastDrive = 0.8;
        public static double MidDrive = 0.5;
        public static double SlowDrive = 0.3;

        public static double RollerSpeed = 0.5;
        public static double OuttakeSpeed = 0.5;

        public static double HandRotationSpeed = 0.3;

    }

    public static void ConstInit() {
        
    }
}
