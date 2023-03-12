package frc.robot.mode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class Mode {
    static XboxController driveController, operateController;
    static Joystick joystick;
    public static void addController(XboxController driveController, XboxController operateController, Joystick joystick) {
        Mode.driveController = driveController;
        Mode.operateController = operateController;
        Mode.joystick = joystick;

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
