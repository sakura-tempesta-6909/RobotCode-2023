package frc.robot.subClass;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;

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
        sendConsole("armActualHeight", State.Arm.actualHeight);
        sendConsole("armActualDepth", State.Arm.actualDepth);
        sendConsole("armActualRootAngle", State.Arm.actualRootAngle);
        sendConsole("armActualJointAngle", State.Arm.actualJointAngle);
        sendConsole("armTargetHeight", State.Arm.targetHeight);
        sendConsole("armTargetDepth", State.Arm.targetDepth);
        sendConsole("armTargetRootAngle", State.Arm.targetRootAngle);
        sendConsole("armTargetJointAngle", State.Arm.targetJointAngle);
        sendConsole("isAtSetpoint", State.Arm.isArmAtTarget);
        sendConsole("controlMode", State.Arm.state.toString());
    }
}
