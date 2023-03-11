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
    public static final Joystick driveJoystick = new Joystick(0);
    public static final JoystickButton Button1 = new JoystickButton(joystick, 1);
    public static final JoystickButton Button2 = new JoystickButton(joystick, 2);
    public static final JoystickButton Button3 = new JoystickButton(joystick, 3);
    public static final JoystickButton Button4 = new JoystickButton(joystick, 4);
    public static final JoystickButton Button5 = new JoystickButton(joystick, 5);
    public static final JoystickButton Button6 = new JoystickButton(joystick, 6);
    public static final JoystickButton Button7 = new JoystickButton(joystick, 7);
    public static final JoystickButton Button8 = new JoystickButton(joystick, 8);
    public static final JoystickButton Button9 = new JoystickButton(joystick, 9);
    public static final JoystickButton Button10 = new JoystickButton(joystick, 10);
    public static final JoystickButton Button11 = new JoystickButton(joystick, 11);
    public static final JoystickButton Button12 = new JoystickButton(joystick, 12);

    /**
     * Modeを変化させる。
     */
    abstract public void changeMode();

    /**
     * Stateを変化させる
     */
    abstract public void changeState();
}
