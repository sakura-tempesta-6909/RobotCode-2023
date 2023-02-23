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

    public static final class Arm {
        public static final class Ports {
            public static final int topMotor = 0;
            public static final int underMotor = 0;
        }

        /**  */
        public static final double RootArmLength = 90.0; // [cm]
        public static final double HeadArmLength = 90.0; // [cm]
        public static final double RootArmBarycenter = 45.0; // [cm]
        public static final double HeadArmBarycenter = 45.0; // [cm]
        public static final double RootArmMass = 0.0 * 9.8; // 注意　[N]=[kg*9.8]
        public static final double HeadArmMass = 0.0 * 9.8; // 注意　[N]=[kg*9.8]
        public static final double TargetModifyRatio = 1;
        public static final double TakeUpLengthAfterGrab = 20.0; // [cm]
        public static final double P_1 = 0.2;
        public static final double I_1 = 0.2;
        public static final double D_1 = 0.01;

        public static final double P_2 = 0.1;
        public static final double I_2 = 0.000;
        public static final double D_2 = 0.000;
        public static final double MotorMaxTorque = 2.6 * 100; // [N*cm] = [N*m] * 100
        public static final double TargetPositionOuterLimit = RootArmLength + HeadArmLength - 2; // [cm]
        public static final double TargetPositionInnerLimit = RootArmLength - HeadArmLength + 2; // [cm]
        public static final double TopMotorGearRatio = 100;
        public static final double UnderMotorGearRatio = 114;
        public static final double PIDAngleTolerance = 0.1; // [deg]
    }

    public static void ConstInit() {

    }
}
