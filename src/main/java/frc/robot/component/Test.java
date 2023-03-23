package frc.robot.component;

import frc.robot.subClass.Const;

public class Test {
    public static void main(String[] args) {
        Test test = new Test();
//        double rotation = 0;
//        double angle = rotation / Const.Arm.RootMotorGearRatio * 360 + Const.Arm.RootHomePosition;

        double angle = test.calculateRootAngleFromRotation(Const.Arm.RootMotorGearRatio);
        System.out.println(test.calculateRootRotationFromAngle(angle));
        System.out.println(angle);

    }
    /**
     * 根本NEOモーターの回転数から根本アームの角度を計算（根本の回転数 * 360）
     * @param rotation encoderから取得したPosition（モーターの回転数）
     * @return 根本アームの角度[deg]
     * */
    public double calculateRootAngleFromRotation(double rotation) {
        System.out.println(rotation);
        return rotation / Const.Arm.RootMotorGearRatio * 360 + Const.Arm.RootHomePosition;

    }

    /**
     * 根本アームの角度から根本NEOモーターの回転数を計算（根本NEOモーターの必要な角度 / 360 = 必要な回転数）
     * @param angle 根本アームの角度[deg]
     * @return 根本NEOモーターの回転数
     * */
    public double calculateRootRotationFromAngle(double angle) {
        double a  = 360/Const.Arm.RootMotorGearRatio;
        double b = Const.Arm.RootHomePosition;
        double x = (-angle + b) / a;
        System.out.println("x:" + x);
        double rotation = angle / (360 / angle) + Const.Arm.RootHomePosition;
        return angle * Const.Arm.RootMotorGearRatio / 360 + rotation;
    }

}
