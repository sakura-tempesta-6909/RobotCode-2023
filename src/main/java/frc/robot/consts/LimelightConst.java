package frc.robot.consts;

public class LimelightConst {
    // limelightの情報
    /**
     * Limelightの横の角度の最大
     */
    public static final double LimelightMaxAngleWidth = 27;

    /**
     * Limelightの角度(度数法)
     */
    public static final double LimelightMountAngleDegrees = -8;

    /**
     * Limelightの高さ
     */
    public static final double LimelightLensHeight = 96.5; //  [cm]

    /**
     * LimelightからArmまでの距離
     */
    public static final double LimelightToArm = 10; // [cm]


    // ターゲットの情報
    /**
     * ターゲットの高さ
     */
    public static final double GoalHeight = 87 - 20 - 5; // [cm]


    /**
     * 前のコーンのゴールの高さ[cm] -> ポールの先端（床の面）の高さは13[cm]
     */
    public static final double BottomGoalHeight = 13 + 30;
    /**
     * 真ん中のコーンのゴールの高さ[cm] -> ポールの先端の高さは87[cm]
     */
    public static final double MiddleGoalHeight = 87 + 35;
    /**
     * 奥のコーンのゴールの高さ[cm] -> ポールの先端の高さは117[cm]
     */
    public static final double TopGoalHeight = 117 + 40;

    /**
     * 手前から奥のターゲットまでの距離
     */
    public static final double FrontGoalToBackGoal = 43; // [cm]

    public static final double SubStationHeight = 95;

    public static final class PID {
        public static final double LimelightDriveP = 0.08;
        public static final double LimelightDriveI = 0.02;
        public static final double LimelightDriveD = 0;
    }

    public static void LimelightConstInit() {

    }
}
