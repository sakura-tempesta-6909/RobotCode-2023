package frc.robot.States;

import edu.wpi.first.networktables.NetworkTable;

public class LimelightState {
    public static NetworkTable table;
    public static boolean isLimelightOn;
    public enum States {

    }

    public static void StateInit() {

    }

    public static void StateReset() {
        isLimelightOn = false;
    }
}
