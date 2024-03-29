package frc.robot.states;

import edu.wpi.first.networktables.NetworkTable;

public class LimelightState {
    public static States limelightState;
    public static NetworkTable table;
    public static boolean isLimelightOn;
    public static boolean isLimelightFlashing;
    public static boolean isConeDetection;
    public static boolean isCubeDetection;
    /** 手前のターゲットまでの距離 */
    public static double limelightToFrontGoal; // [cm]
    public static double limelightToBackGoal; // [cm]
    public static double tx;
    public static boolean tv;
    public static double limelightXSpeed;
    public static boolean pidLimelightReset;
    public static double limelightToCone;
    /** armからターゲットまでの距離 */
    public static double armToGoal; // [cm]

    public static double limelightToCube;

    public static double armToCone;
    public static double armToCube;

    public enum States {
        s_coneDetection,
        s_cubeDetection,
        s_tapeDetection,
    }

    public static void StateInit() {
        limelightState = States.s_tapeDetection;
    }

    public static void StateReset() {
        isLimelightOn = false;
        isLimelightFlashing = false;
        
    }
}
