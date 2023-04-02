package frc.robot.subClass;

import frc.robot.States.State;

import java.util.HashMap;
import java.util.Map;

public class Tools {
    /**
     * 回転行列をかける <a href="https://mathlandscape.com/rotation-matrix/">rotation matrix</a>
     *
     * @param theta 回転する角度[deg]
     * @param x     x座標[cm]
     * @param y     y座標[cm]
     * @return 回転先の位置ベクトル <br>（x_dash,<br>&emsp;y_dash）
     */
    public static Map<Integer, Double> rotateMatrix(double theta, double x, double y) {
        theta = Math.toRadians(theta); // [rad]
        double x_dash = x * Math.cos(theta) - y * Math.sin(theta);
        double y_dash = x * Math.sin(theta) + y * Math.cos(theta);
        Map<Integer, Double> vector = new HashMap<>();
        vector.put(0, x_dash);
        vector.put(1, y_dash);
        return vector;
    }

    /**
     * @param theta_r readSensorで取得した実際の角度[deg]
     * @param theta_j readSensorで取得した実際の角度[deg]
     * @return Depth座標[cm]
     */
    public static double calculateDepth(double theta_r, double theta_j) {
        Map<Integer, Double> positionVec = calculatePositionVec(theta_r, theta_j);
        return positionVec.get(0);
    }

    /**
     * @param theta_r RootAngle - readSensorで取得した実際の角度[deg]
     * @param theta_j JointAngle - readSensorで取得した実際の角度[deg]
     * @return Height座標[cm]
     */
    public static double calculateHeight(double theta_r, double theta_j) {
        Map<Integer, Double> positionVec = calculatePositionVec(theta_r, theta_j);
        return positionVec.get(1);
    }

    /**
     * @param theta_r readSensorで取得した実際の角度[deg]
     * @param theta_j readSensorで取得した実際の角度[deg]
     * @return アームの先端の座標の位置ベクトル <br>
     * 0 - X座標 - 奥行き（Depth）[cm]<br>
     * 1 - Y座標 - 高さ（Height）[cm]<br>
     */
    public static Map<Integer, Double> calculatePositionVec(double theta_r, double theta_j) {
        double theta_c = Const.Arm.HandFoldAngle; // [deg]

        double l_r = Const.Arm.RootArmLength;
        double l_j = Const.Arm.HeadArmLength;
        double l_h = Const.Arm.HandLength;

        Map<Integer, Double> rootVec = rotateMatrix(theta_r, l_r, 0.0);
        Map<Integer, Double> headVec = rotateMatrix(theta_r + theta_j, l_j, 0.0);
        Map<Integer, Double> handVec = rotateMatrix(theta_c + theta_r + theta_j, l_h, 0.0);

        Map<Integer, Double> positionVec = new HashMap<>();
        positionVec.put(0, rootVec.get(0) + headVec.get(0) + handVec.get(0));
        positionVec.put(1, rootVec.get(1) + headVec.get(1) + handVec.get(1));

        return positionVec;
    }

    /** コントローラーの不感帯の大きさ（絶対値）[0.0) */
    private static final double deadZoneThreshold = 0.1;

    /**
     * 不感帯処理関数
     * 絶対値がdeadZoneThreshold未満のものを淘汰
     *
     * @param input コントローラーの値を入力
     * @return 不感帯処理を施したinput
     */
    public static double deadZoneProcess(double input) {
        if (Math.abs(input) < deadZoneThreshold) return 0.0;
        else return input;
    }

    /**
     * @param x ターゲットの奥行き（Depth）[cm]
     * @param y ターゲットの高さ（Height）[cm]<br>
     *          Target[Depth/Height]からTarget[Root/Joint]Angleを計算
     *          関数内はすべて[rad] 出力はすべて[deg]で統一
     * @return アームのターゲットの角度[deg]
     */
    public static Map<String, Double> calculateAngles(double x, double y) {
        double theta_j_pm;
        if (x >= 0 & y >= 0) {
            // 第1象限
            theta_j_pm = -1;
        } else if (x < 0 & y >= 0) {
            // 第2象限
            theta_j_pm = 1;
        } else if (x < 0 & y < 0) {
            // 第3象限
            theta_j_pm = -1;
        } else {
            // 第4象限
            theta_j_pm = -1;
        }

        double l_r = Const.Arm.RootArmLength;
        double l_v = Const.Arm.VirtualHeadArmLength;

        double theta_h = Math.toRadians(Const.Arm.VirtualArmFoldAngle); // [rad]
        double theta_j = theta_j_pm * Math.acos((Math.pow(x, 2) + Math.pow(y, 2)
                - Math.pow(l_r, 2) - Math.pow(l_v, 2)) / (2 * l_r * l_v)) // [rad]
                - theta_h; // [rad]
        double theta_arg_target = Math.atan2(
                y,
                x
        ); // [rad]
        double theta_arg_zero = Math.atan2(
                l_v * Math.sin(theta_j + theta_h),
                l_r + l_v * Math.cos(theta_j + theta_h)
        ); // [rad]
        double theta_r = theta_arg_target - theta_arg_zero; // [rad]

        Map<String, Double> angles = new HashMap<>();
        angles.put("RootAngle", Math.toDegrees(theta_r));
        angles.put("JointAngle", Math.toDegrees(theta_j));

        return angles; // [deg]
    }

    /**
     * @param RootAngle  readSensorで取得した実際の角度[deg]
     * @param JointAngle readSensorで取得した実際の角度[deg]
     *                   underMotorのフィードフォワードを計算
     *                   それぞれのアームの重心をConstから取得
     * @return モーメント[N*cm]
     */
    public static double calculateRootMotorFeedforward(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.RootArmLength;
        double l2 = Const.Arm.HeadArmLength;
        double b1 = Const.Arm.RootArmBarycenter; //FirstBarycenter -> fb
        double b2 = Const.Arm.HeadArmBarycenter; //SecondBarycenter -> sb
        double m1 = Const.Arm.RootArmMass;
        double m2 = Const.Arm.HeadArmMass;

        double ffMomentForRootArm = b1 * m1 * Math.cos(RootAngle);
        double ffMomentForHeadArm = l1 * m2 * Math.cos(RootAngle) + b2 * m2 * Math.cos(SumAngle);

        //TODO feedforwardでmotor.setに渡す値はトルクの計算が必要
        return (ffMomentForRootArm * Const.Arm.RootArmFFWeightForRM
                + ffMomentForHeadArm * Const.Arm.HeadArmFFWeightForRM)
                * Const.Arm.RootMotorFFWeight;
    }

    /**
     * @param RootAngle  readSensorで取得した実際の角度[deg]
     * @param JointAngle readSensorで取得した実際の角度[deg]
     *                   topMotorのフィードフォワードを計算
     *                   重心などをConstから取得
     * @return モーメント[N*cm]
     */
    public static double calculateJointMotorFeedforward(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double b2 = Const.Arm.HeadArmBarycenter; //SecondBarycenter -> sb
        double m2 = Const.Arm.HeadArmMass;

        return (b2 * m2 * Math.cos(SumAngle)) * Const.Arm.JointMotorFFWeight;
    }

    /**
     * NEOモーターのトルクとRPMの関係を利用 <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシート</a>
     * [注意] NEOモーターに合わせて出力する
     *
     * @param torque トルク[N*cm] = モーメント / Const.Arms.[Under/Top]MotorGearRatio（ギア比に合わせて入力）
     * @return motor.setへの入力[-1.0, 1.0] (CANSparkMax)
     */
    public static double changeTorqueToMotorInput(double torque) {
        return torque / Const.Arm.MotorMaxTorque;
        // TODO 2次関数的にトルクを求める必要があるらしい？
    }

    public static void main(String[] args) {
        State.StateReset();

        double targetDepth = -10;//State.Arm.TargetDepth.TopCorn;
        double targetHeight = -60;//Const.Calculation.Limelight.TopGoalHeight - Const.Arm.RootHeightFromGr;
        System.out.println(targetDepth);
        System.out.println(targetHeight);
        System.out.println(isNewTargetPositionInLimit(targetHeight, targetDepth));
        Map<String, Double> map = calculateAngles(targetDepth, targetHeight);
        System.out.println(map);
//        double joint = map.get("JointAngle");
//        double root = map.get("RootAngle");
//        double joint = -90;
//        double root = 0;
//        System.out.println(calculateDepth(root, joint));
//        System.out.println(calculateHeight(root, joint));

        double l_r = Const.Arm.RootArmLength;
        double l_j = Const.Arm.HeadArmLength;
        double l_h = Const.Arm.HandLength;

        double theta_r = 0;
        double theta_j = 0;
        double theta_c = Const.Arm.HandFoldAngle;

        Map<Integer, Double> vec = rotateMatrix(theta_c + theta_r + theta_j, l_h, 0.0);
        System.out.println(vec);

        int correct = 0;
        int error = 0;
        int all = 0;
        for (double root = -180; root <= 180; root += 1) {
            for (double joint = -180; joint <= 180; joint += 1) {
                all++;
                Map<String, Double> angles = calculateAngles(calculateDepth(root, joint), calculateHeight(root, joint));
                double p_root = angles.get("RootAngle");
                double p_joint = angles.get("JointAngle");
                if (isLossInTolerance(calculateDepth(root, joint), calculateDepth(p_root, p_joint), 0.1) && isLossInTolerance(calculateHeight(root, joint), calculateHeight(p_root, p_joint), 0.1)) {
                    correct++;
                } else {
                    System.out.println(root);
                    System.out.println(joint);
                    error++;
                }
            }
        }

        System.out.printf("Correct : %d / %d\n", correct, all);
        System.out.printf("Incorrect : %d / %d\n", error, all);
        System.out.printf("CorrectRate : %d\n", correct / all);
        System.out.printf("IncorrectRate : %d\n", error / all);
    }

    private static boolean isLossInTolerance(double original, double predicted, double tolerance) {
        return Math.abs(original - predicted) < tolerance;
    }

    private static boolean isNewTargetPositionInLimit(double Height, double Depth) {
        double length = Math.sqrt(Math.pow(Height, 2) + Math.pow(Depth, 2));

        boolean isInOuterBorder = length < Const.Arm.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > Const.Arm.TargetPositionInnerLimit;
        boolean isInDepthLimit = Depth > -23;

        // TODO XButtonでコントロールする時のターゲット座標の制限を考える
        return isInOuterBorder && isOutInnerBorder && isInDepthLimit;
    }
}
