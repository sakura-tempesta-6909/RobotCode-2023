package frc.robot.subClass;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Util {
    public static void sendSystemOut(PrintStream defaultConsole, ByteArrayOutputStream newConsole){
        defaultConsole.print(newConsole);
    }
    public static void sendConsole(String key, Boolean which){
        // System.out.println(key + ":" +which);
        SmartDashboard.putBoolean(key, which);
    }

    public static void allSendConsole(){
    }
}
