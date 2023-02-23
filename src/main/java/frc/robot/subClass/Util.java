package frc.robot.subClass;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;

public class Util {
    public static void sendSystemOut(PrintStream defaultConsole, ByteArrayOutputStream newConsole){
        defaultConsole.print(newConsole);
    }

    public static void sendConsole(String key, String text) {
        // System.out.println(key + ":" + text);
        SmartDashboard.putString(key, text);
    }

    public static void sendConsole(String key, double number) {
        // System.out.println(key + ":" +number);
        SmartDashboard.putNumber(key, number);
    }
    public static void sendConsole(String key, Boolean which){
        // System.out.println(key + ":" +which);
        SmartDashboard.putBoolean(key, which);
    }

    public static void allSendConsole(){
        sendConsole("armActualHeight", State.armActualHeight);
        sendConsole("armActualDepth", State.armActualDepth);
        sendConsole("armActualRootAngle", State.armActualRootAngle);
        sendConsole("armActualJointAngle", State.armActualJointAngle);
        sendConsole("armTargetHeight", State.armTargetHeight);
        sendConsole("armTargetDepth", State.armTargetDepth);
        sendConsole("armTargetRootAngle", State.armTargetRootAngle);
        sendConsole("armTargetJointAngle", State.armTargetJointAngle);
        sendConsole("isAtSetpoint", State.isArmAtTarget);
        sendConsole("controlMode", State.armState.toString());
    }
}
