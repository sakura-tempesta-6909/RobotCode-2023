package frc.robot.mode;

import edu.wpi.first.wpilibj.XboxController;

public abstract class Mode {
    static XboxController driveController, operateController;
    public static void addController(XboxController driveController, XboxController operateController) {
        Mode.driveController = driveController;
        Mode.operateController = operateController;

    }

    /**
     * Modeを変化させる。
     */
    abstract public void changeMode();

    /**
     * Stateを変化させる
     */
    abstract public void changeState();
}
