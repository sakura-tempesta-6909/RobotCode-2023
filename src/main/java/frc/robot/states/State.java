package frc.robot.states;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.mode.*;
import frc.robot.subClass.Const;

import java.util.HashMap;
import java.util.Map;

public class State {
    public static Modes mode;
    /** Autonomousの遷移の種類　[ A, B, C ] のいずれか */
    public static String autonomousPhaseTransType = "C";



    public static Map<String, Double> voltage = new HashMap<>();

    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Joystick joystick = new Joystick(Const.Ports.Joystick);

        State.mode = State.Modes.k_drive;

        Mode.addController(driveController, operateController, joystick);
        LimelightState.StateInit();

        voltage = new HashMap<>();

        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        LimelightState.StateReset();
    }



    public enum Modes {
        k_drive(new DriveMode()),
        k_arm(new ArmMode()),
        k_test(new TestMode()),
        k_config(new ConfigMode()),
        k_chargeStation(new ChargeStationMode());

        private final Mode mode;

        Modes(Mode mode) {
            this.mode = mode;
        }

        public void changeMode() {
            this.mode.changeMode();
        }

        public void changeState() {
            this.mode.changeState();
        }
    }
}
