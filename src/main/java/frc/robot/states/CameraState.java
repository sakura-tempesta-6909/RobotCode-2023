package frc.robot.states;

public class CameraState {
    /** cameraからtagまでの距離 */
    public static double cameraToTag; // [cm]
    /** cameraからみたaprilTagの縦の角度(度数法) */
    public static double aprilTagAngleHeight;
    /** cameraからみたaprilTagの横の角度(度数法) */
    public static double aprilTagAngleWidth;
    /** カメラの横の中心座標 */
    public static double cameraCenterWidth;
    /** カメラの盾の中心座標 */
    public static double cameraCenterHeight;
    public static double cameraXSpeed;
    /** armからtagまでの距離 */
    public static double armToTag; // [cm]
    public enum States {

    }

    public static void StateInit() {

    }

    public static void StateReset() {

    }
}
