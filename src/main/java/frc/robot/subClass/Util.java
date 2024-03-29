package frc.robot.subClass;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.ArmState;
import frc.robot.states.DriveState;
import frc.robot.states.HandState;
import frc.robot.states.State;
import frc.robot.consts.ArmConst;
import frc.robot.consts.DriveConst;

public class Util {

    public static String getConsole(String key, String defaultText) {
        return SmartDashboard.getString(key, defaultText);
    }

    public static void sendConsole(String key, String text) {
        // System.out.println(key + ":" + text);
        SmartDashboard.putString(key, text);
    }

    public static void sendConsole(String key, double number) {
        // System.out.println(key + ":" +number);
        SmartDashboard.putNumber(key, number);
    }

    public static void sendConsole(String key, Boolean which) {
        // System.out.println(key + ":" +which);
        SmartDashboard.putBoolean(key, which);
    }

    public static void allSendConsole() {
        sendConsole("MODE", State.mode.toString());
        sendConsole("ArmMode", ArmState.armState.toString());
        sendConsole("armActualHeight", ArmState.actualHeight);
        sendConsole("armActualDepth", ArmState.actualDepth);
        sendConsole("armActualRootAngle", ArmState.actualRootAngle);
        sendConsole("armActualJointAngle", ArmState.actualJointAngle);
        sendConsole("armTargetHeight", ArmState.targetHeight);
        sendConsole("armTargetDepth", ArmState.targetDepth);
        sendConsole("armTargetRootAngle", ArmState.targetRootAngle);
        sendConsole("armTargetJointAngle", ArmState.targetJointAngle);
        sendConsole("isAtSetpoint", ArmState.isAtTarget());
        sendConsole("controlMode", ArmState.armState.toString());
        sendConsole("rootff", ArmState.rootMotorFeedforward);
        sendConsole("jointff", ArmState.jointMotorFeedforward);
        sendConsole("handAngle", HandState.actualHandAngle % 360);

        sendConsole("DriveTargetMeter", DriveState.targetMeter);
        sendConsole("RDrivePosition", DriveState.rightMeter);
        sendConsole("LDrivePosition", DriveState.leftMeter);

        sendConsole("HandIsAtTarget", HandState.isAtTarget());
        sendConsole("DriveIsAtTarget", DriveState.isAtTarget());
    }

    public static class Calculate {
        /**
         * @param points ドライブのエンコーダーの値
         * @return ドライブのエンコーダーの値をメートルに変換した値
         */
        public static double driveEncoderPointsToMeter(double points) {
            return points / DriveConst.PID.PointsPerLength;
        }

        /**
         * @param meter メートル
         * @return メートルをドライブのエンコーダーのポジションに変換した値
         */
        public static double meterToDriveEncoderPoints(double meter) {
            return meter * DriveConst.PID.PointsPerLength;
        }

        /**
         *
         * @param actualHeight 実際の高さ
         * @param actualDepth　実際の奥行き
         * @return 中継地点に到達しているかどうか
         */
        public static boolean isOverRelayToGoal(double actualHeight, double actualDepth) {
            return actualHeight > ArmConst.RelayPointToGoalHeight - ArmConst.RelayPointTolerance && actualDepth > ArmConst.RelayPointToGoalDepth - ArmConst.RelayPointTolerance;
        }

        /**
         *
         * @param actualHeight 実際の高さ
         * @param actualDepth　実際の奥行き
         * @return 中継地点に到達しているかどうか
         */
        public static boolean isOverRelayToInit(double actualHeight, double actualDepth) {
            return actualHeight < ArmConst.RelayPointToInitHeight + ArmConst.RelayPointTolerance && actualDepth < ArmConst.RelayPointToInitDepth + ArmConst.RelayPointTolerance;
        }

        public static boolean isOverTargetToGoal() {
            return ArmState.isAtTarget();
        }

        public static void setInitWithRelay() {
            if (ArmState.relayToInitOver) {
                ArmState.targetHeight = ArmConst.InitialHeight;
                ArmState.targetDepth = ArmConst.InitialDepth;
            } else {
                ArmState.targetHeight = ArmConst.RelayPointToInitHeight;
                ArmState.targetDepth = ArmConst.RelayPointToInitDepth;
            }
        }
    }
}
