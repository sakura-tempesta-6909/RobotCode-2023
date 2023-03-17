package frc.robot.subClass;

import frc.robot.State;

import java.util.HashMap;
import java.util.Map;

public class Tools {
    /**
     * @param RootAngle readSensorで取得した実際の角度[deg]
     * @param JointAngle readSensorで取得した実際の角度[deg]
     * @return Height座標[cm]
     * */
    public static double calculateHeight(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.RootArmLength;
        double l2 = Const.Arm.HeadArmLength;

        return l1 * Math.sin(RootAngle) + l2 * Math.sin(SumAngle);
    }

    /**
     * @param RootAngle readSensorで取得した実際の角度[deg]
     * @param JointAngle readSensorで取得した実際の角度[deg]
     * @return Depth座標[cm]
     * */
    public static double calculateDepth(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.RootArmLength;
        double l2 = Const.Arm.HeadArmLength;

        return l1 * Math.cos(RootAngle) + l2 * Math.cos(SumAngle);
    }

    /** コントローラーの不感帯の大きさ（絶対値）[0.0) */
    private static final double deadZoneThreshold = 0.1;

    /**
     * 不感帯処理関数
     * 絶対値がdeadZoneThreshold未満のものを淘汰
     * @param input コントローラーの値を入力
     * @return 不感帯処理を施したinput
     * */
    public static double deadZoneProcess(double input) {
        if(Math.abs(input) < deadZoneThreshold) return 0.0;
        else return input;
    }

    /**
     * @param Depth ターゲットのX座標[cm]
     * @param Height ターゲットのY座標[cm]
     * X,YからRootAngle, JointAngle を計算
     * @return アームのターゲットの角度[deg]
     */
    public static Map<String, Double> calculateAngles(double Depth, double Height) {
        double l1 = Const.Arm.RootArmLength;
        double l2 = Const.Arm.HeadArmLength;

        double DepthPM = Math.signum(Depth);
        double HeightPM = Math.signum(Height);

        double JointAngle = Math.acos(Math.min((Math.pow(Height, 2) + Math.pow(Depth, 2)
                - Math.pow(l1, 2) - Math.pow(l2, 2)) / (2 * l1 * l2), 1.0));

        double tX = l1 + l2 * Math.cos(JointAngle);
        double tY = l2 * Math.sin(JointAngle);
        double r = Math.sqrt(Math.pow(tX, 2) + Math.pow(tY, 2));

        double alpha = Math.acos(tX / r);

        Map<String, Double> angles = new HashMap<String, Double>();

        // theta2 < 0の時
        double RootAngleM = Math.acos(Math.max(Math.min(Depth / r, 1.0), -1.0)) + alpha;
        // theta2 > 0の時
        double RootAngleP = Math.acos(Math.max(Math.min(Depth / r, 1.0), -1.0)) - alpha;

        if (HeightPM >= 0 && DepthPM >= 0) {
            angles.put("RootAngle", Math.toDegrees(RootAngleM));
            angles.put("JointAngle", Math.toDegrees(-1 * JointAngle));
        } else if (HeightPM < 0 && DepthPM >= 0) {
            angles.put("RootAngle", Math.toDegrees(-1 * RootAngleP));
            angles.put("JointAngle", Math.toDegrees(-1 * JointAngle));
        } else if (HeightPM < 0 && DepthPM < 0) {
            angles.put("RootAngle", Math.toDegrees(RootAngleM) - 180);
            angles.put("JointAngle", -1 * Math.toDegrees(JointAngle));
        } else {
            angles.put("RootAngle", 180 - Math.toDegrees(RootAngleP));
            angles.put("JointAngle", -1 * Math.toDegrees(JointAngle));
        }

        return angles;
    }

    /**
     * @param RootAngle readSensorで取得した実際の角度[deg]
     * @param JointAngle readSensorで取得した実際の角度[deg]
     * underMotorのフィードフォワードを計算
     * それぞれのアームの重心をConstから取得
     * @return モーメント[N*cm]
     * */
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
     * @param RootAngle readSensorで取得した実際の角度[deg]
     * @param JointAngle readSensorで取得した実際の角度[deg]
     * topMotorのフィードフォワードを計算
     * 重心などをConstから取得
     * @return モーメント[N*cm]
     * */
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
     * @param torque トルク[N*cm] = モーメント / Const.Arms.[Under/Top]MotorGearRatio（ギア比に合わせて入力）
     * @return motor.setへの入力[-1.0, 1.0] (CANSparkMax)
     * */
    public static double changeTorqueToMotorInput (double torque) {
        return torque / Const.Arm.MotorMaxTorque;
        // TODO 2次関数的にトルクを求める必要があるらしい？
    }

    public static void main(String[] args) {
        State.StateReset();
        double targetDepth = 0;//State.Arm.TargetDepth.TopCorn;
        double targetHeight = 0;//Const.Calculation.Limelight.TopGoalHeight - Const.Arm.RootHeightFromGr;
        System.out.println(targetDepth);
        System.out.println(targetHeight);
        System.out.println(isNewTargetPositionInLimit(targetHeight, targetDepth));
        Map<String, Double> map = calculateAngles(targetDepth, targetHeight);
        System.out.println(map);
        double joint = map.get("JointAngle");
        double root = map.get("RootAngle");
        System.out.println(calculateDepth(root, joint));
        System.out.println(calculateHeight(root, joint));
    }

    private static boolean isNewTargetPositionInLimit(double Height, double Depth) {
        double length = Math.sqrt(Math.pow(Height, 2) + Math.pow(Depth, 2));

        boolean isInOuterBorder = length < Const.Arm.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > Const.Arm.TargetPositionInnerLimit;

        // TODO XButtonでコントロールする時のターゲット座標の制限を考える
        return isInOuterBorder && isOutInnerBorder;
    }
}
