package frc.robot.states;

public class IntakeState {
    public static RollerStates intakeState;
    public static IntakeExtensionStates intakeExtensionState;
    public static boolean isCompressorEnable;

    public enum RollerStates {
        /** Rollerを外向きに動かし、ゲームピースを出す */
        s_outtakeGamePiece,
        /** Rollerを内向きに動かし、ゲームピースを取り込む */
        s_intakeGamePiece,
        /** Rollerの動きを止める */
        s_stopRoller,

    }
    public enum IntakeExtensionStates {
        /** Intakeを出す */
        s_openIntake,
        /** Intakeをしまう */
        s_closeIntake,
    }

    public static void StateInit() {
        intakeExtensionState = IntakeExtensionStates.s_openIntake;

    }

    public static void StateReset() {
        intakeState = RollerStates.s_stopRoller;
        isCompressorEnable = true;
    }
}
