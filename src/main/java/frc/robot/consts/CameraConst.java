package frc.robot.consts;

public class CameraConst {
    //　Cameraの情報
    /**
     * 　Cameraの縦の中心
     */
    public static final double CameraCenterHeight = 320;

    /**
     * Cameraの横の中心
     */
    public static final double CameraCenterWidth = 240;

    /**
     * Cameraの横の比
     */
    public static final double VerticalRatio = 4;

    /**
     * Cameraの縦の比
     */
    public static final double HorizontalRatio = 3;

    /**
     * Cameraの視野角の半分
     */
    public static final double FieldOfViewHalf = 34.25;

    /**
     * Cameraの角度(度数法)
     */
    public static final double CameraMountAngleDegrees = 0;

    /**
     * Cameraの高さ
     */
    public static final double CameraLensHeight = 99; // [cm]

    /**
     * CameraからArmまでの距離
     */
    public static final double CameraToArm = 10; // [cm]


    //ターゲットの情報
    /**
     * ターゲットの高さ
     */
    public static final double GoalHeight = 76.5; // [cm]


    /**
     * 奥のキューブのゴールの高さ[cm] -> ゴールの面の高さは90[cm]
     */
    public static final double TopGoalHeight = 90 + 40
    ;
    /**
     * 真ん中のキューブのゴールの高さ[cm] -> ゴールの面の高さは60[cm]
     */
    public static final double MiddleGoalHeight = 60 + 20;
    /**
     * 前のキューブのゴールの高さ[cm] -> ゴールの面の高さは13[cm]
     */
    public static final double BottomGoalHeight = 13 + 25;


    //計算
    /**
     * 斜辺の長さ
     */
    public static final double ObliqueLine = Math.pow(VerticalRatio, 2) + Math.pow(HorizontalRatio, 2);

    /**
     * 縦の角度の最大
     */
    public static final double ThetaMaxHeight = FieldOfViewHalf * VerticalRatio / Math.sqrt(ObliqueLine);

    /**
     * 横の角度の最大
     */
    public static final double ThetaMaxWidth = FieldOfViewHalf * HorizontalRatio / Math.sqrt(ObliqueLine);

    /**
     * 縦の焦点距離
     */
    public static final double FocalLengthHeight = CameraCenterHeight / Math.tan(Math.toRadians(ThetaMaxHeight));
    /**
     * 横の焦点距離
     */
    public static final double FocalLengthWeight = CameraCenterWidth / Math.tan(Math.toRadians(ThetaMaxWidth));

    public static final class PID {
        public static final double CameraDriveP = 0.05;
        public static final double CameraDriveI = 0.0007;
        public static final double CameraDriveD = 0.003;
    }
    public static void CameraConstInit() {

    }
}
