package frc.robot.subClass;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.States.State;

public class Util {

    public static String getConsole(String key) {
        return SmartDashboard.getString(key, "None");
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
        sendConsole("ArmMode", State.Arm.state.toString());
        sendConsole("armActualHeight", State.Arm.actualHeight);
        sendConsole("armActualDepth", State.Arm.actualDepth);
        sendConsole("armActualRootAngle", State.Arm.actualRootAngle);
        sendConsole("armActualJointAngle", State.Arm.actualJointAngle);
        sendConsole("armTargetHeight", State.Arm.targetHeight);
        sendConsole("armTargetDepth", State.Arm.targetDepth);
        sendConsole("armTargetRootAngle", State.Arm.targetRootAngle);
        sendConsole("armTargetJointAngle", State.Arm.targetJointAngle);
        sendConsole("isAtSetpoint", State.Arm.isAtTarget());
        sendConsole("controlMode", State.Arm.state.toString());
        sendConsole("rootff", State.Arm.rootMotorFeedforward);
        sendConsole("jointff", State.Arm.jointMotorFeedforward);
        sendConsole("handAngle", State.Hand.actualHandAngle % 360);

        sendConsole("DriveTargetMeter", State.Drive.targetMeter);
        sendConsole("RDrivePosition", State.Drive.rightMeter);
        sendConsole("LDrivePosition", State.Drive.leftMeter);
    }

    public static class Calculate {
        /**
         * @param points ドライブのエンコーダーの値
         * @return ドライブのエンコーダーの値をメートルに変換した値
         */
        public static double driveEncoderPointsToMeter(double points) {
            return points / Const.Calculation.Drive.DrivePointsPerDriveLength;
        }

        /**
         * @param meter メートル
         * @return メートルをドライブのエンコーダーのポジションに変換した値
         */
        public static double meterToDriveEncoderPoints(double meter) {
            return meter * Const.Calculation.Drive.DrivePointsPerDriveLength;
        }

        public static boolean relayJudge(double actual, double relay) {
            return actual < relay;
        }
    }
}
