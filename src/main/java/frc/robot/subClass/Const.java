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
        public static final class Limelight {
            public static final double LimelightMaxHeight = 27;
            public static final double LimelightMaxWidth = 20.5;
            //Limelightの角度
            public static final double LimelightMountAngleDegrees = 34.5;
            //Limelightの高さ
            public static final double LimelightLensHeightInCM = 81.5;
            //ターゲットの高さ
            public static final double GoalHeightInCM = 166;

        }


        public static final class Camera {
            public static final double CameraCenterHeight = 320;
            public static final double CameraCenterWidth = 240;
            public static final double VerticalRatio = 4;
            public static final double HorizontalRatio = 3;
            public static final double FieldOfViewHalf = 34.25;
            public static final double ObliqueLine = Math.pow(VerticalRatio, 2) + Math.pow(HorizontalRatio, 2);
            public static final double ThetaMaxHeight = FieldOfViewHalf * VerticalRatio / Math.sqrt(ObliqueLine);
            public static final double ThetaMaxWidth = FieldOfViewHalf * HorizontalRatio / Math.sqrt(ObliqueLine);

            public static final double FocalLengthHeight = CameraCenterHeight / Math.tan(Math.toRadians(ThetaMaxHeight));
            public static final double FocalLengthWeight = CameraCenterWidth / Math.tan(Math.toRadians(ThetaMaxWidth));

            public static final double CameraMountAngleDegrees = 0;
            public static final double CameraLensHeightInCM = 42.5;
            public static final double GoalHeightInCM = 76.5;
        }


    }


    public static void ConstInit() {

    }
}
