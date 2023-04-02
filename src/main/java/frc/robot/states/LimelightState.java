package frc.robot.states;

import edu.wpi.first.networktables.NetworkTable;

public class LimelightState {
    public static NetworkTable table;
    public static boolean isLimelightOn;
    /** 手前のターゲットまでの距離 */
    public static double limelightToFrontGoal; // [cm]
    public static double limelightToBackGoal; // [cm]
    public static double tx;
    public static boolean tv;
    public static double limelightXSpeed;
    public static boolean pidLimelightReset;
    /** armからターゲットまでの距離 */
    public static double armToGoal; // [cm]
    public enum States {

    }

    public static void StateInit() {

    }

    public static void StateReset() {
        isLimelightOn = false;
    }
}
