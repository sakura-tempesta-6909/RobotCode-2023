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
        sendConsole("armActualAxisX", State.armActualAxisX);
        sendConsole("armActualAxisZ", State.armActualAxisZ);
        sendConsole("armActualTheta1", State.armActualTheta1);
        sendConsole("armActualTheta2", State.armActualTheta2);
        sendConsole("armTargetAxisX", State.armTargetAxisX);
        sendConsole("armTargetAxisZ", State.armTargetAxisZ);
        sendConsole("armTargetTheta1", State.armTargetTheta1);
        sendConsole("armTargetTheta2", State.armTargetTheta2);
        sendConsole("isAtSetpoint", State.isArmAtTarget);
        sendConsole("controlMode", State.armState.toString());
    }
}
