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

    public static final class Calculation {
        public static double ObliqueLine = Math.pow(4, 2) + Math.pow(3, 2);
        public static double ThetaMaxX = 34.25 * 4 / Math.sqrt(ObliqueLine);
        public static double ThetaMaxY = 34.25 * 3 / Math.sqrt(ObliqueLine);

        public static double FocalLengthX = 80 / Math.tan(Math.toRadians(ThetaMaxX));
        public static double FocalLengthY = 60 / Math.tan(Math.toRadians(ThetaMaxY));

        public static double CameraMountAngleDegrees = 0;
        public static double CameraLensHeightCentis = 42.5;
        public static double GoalHightCentis = 76.5;
        
       
    }

    public static void ConstInit() {
        
    }
}
