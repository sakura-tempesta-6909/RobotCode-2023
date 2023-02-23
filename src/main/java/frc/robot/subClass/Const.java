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
            public static final double LimelightMaxAngleHeight = 27;
            //Limelightの角度
            public static final double LimelightMountAngleDegrees = 34.5;
            //Limelightの高さ
            public static final double LimelightLensHeight = 81.5; //  [cm]
            //ターゲットの高さ
            public static final double GoalHeight = 166; // [cm]
            public static final double DistanceFromLimelightToArm = 0; // [cm]

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
            public static final double CameraLensHeight = 42.5; // [cm]
            public static final double GoalHeight = 76.5; // [cm]
            public static final double DistanceFromCameraToArm = 0; // [cm]

        }


    }

    public static final class Arm {
        public static double FirstArmLength = 90.0; // [cm]
        public static double SecondArmLength = 90.0; // [cm]
        public static double FirstArmBarycenter = 45.0; // [cm]
        public static double SecondArmBarycenter = 45.0; // [cm]
        public static double FirstArmMass = 0.0 * 9.8; // 注意　[N]=[kg*9.8]
        public static double SecondArmMass = 0.0 * 9.8; // 注意　[N]=[kg*9.8]
        public static double TargetModifyRatio = 1;
        public static double TakeUpLengthAfterGrab = 20.0; // [cm]
        public static double kP1 = 0.2;
        public static double kI1 = 0.2;
        public static double kD1 = 0.01;

        public static double kP2 = 0.1;
        public static double kI2 = 0.000;
        public static double kD2 = 0.000;

        public static double Encoder1CountPerRotation = 14.3469;
        public static double Encoder2CountPerRotation = 9.196;
        public static double GearRatio = 140;
        public static double MotorMaxTorque = 2.6 * 100; // [N*cm] = [N*m] * 100
        public static double TargetPositionOuterLimit = FirstArmLength + SecondArmLength - 2; // [cm]
        public static double TargetPositionInnerLimit = FirstArmLength - SecondArmLength + 2; // [cm]
        public static double TopMotorGearRatio = 100;
        public static double TopUnderGearRatio = 114;
    }

    public static void ConstInit() {

    }
}
