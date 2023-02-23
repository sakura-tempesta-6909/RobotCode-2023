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
        public static final double Neutral = 0;

        public static final double FastDrive = 0.8;
        public static final double MidDrive = 0.5;
        public static final double SlowDrive = 0.3;

    }

    public static final class Calculation {
        public static final class Camera {
            public static final double CameraCenterHight = 320;
            public static final double CameraCenterWeight = 240;
            public static final double VerticalRatio = 4;
            public static final double HorizontalRatio = 3;
            public static final double FieldOfViewHalf = 34.25;
            public static final double ObliqueLine = Math.pow(VerticalRatio, 2) + Math.pow(HorizontalRatio, 2);
            public static final double ThetaMaxHight = FieldOfViewHalf * VerticalRatio / Math.sqrt(ObliqueLine);
            public static final double ThetaMaxWeigt = FieldOfViewHalf * HorizontalRatio / Math.sqrt(ObliqueLine);

            public static final double FocalLengthHight = CameraCenterHight / Math.tan(Math.toRadians(ThetaMaxHight));
            public static final double FocalLengthWeight = CameraCenterWeight / Math.tan(Math.toRadians(ThetaMaxWeigt));

            public static final double CameraMountAngleDegrees = 0;
            public static final double CameraLensHeightCentis = 42.5;
            public static final double GoalHightCentis = 76.5;
        }


    }

    public static void ConstInit() {

    }
}
