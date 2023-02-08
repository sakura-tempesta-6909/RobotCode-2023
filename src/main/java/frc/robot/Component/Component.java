package frc.robot.component;

public interface Component {
    /**
     * autonomous時の初期化
     */
    void autonomousInit();

    /**
     * teleop時の初期化
     */
    void teleopInit();

    /**
     * disable時の初期化
     */
    void disabledInit();

    /**
     * test時の初期化
     */
    void testInit();

    /**
     * センサーの値を読む。
     */
    void readSensors();

    /**
     * stateを適用する
     */
    void applyState();
}
