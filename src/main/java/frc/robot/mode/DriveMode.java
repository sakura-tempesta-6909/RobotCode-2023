package frc.robot.mode;

import com.fasterxml.jackson.databind.ser.std.AsArraySerializerBase;

import frc.robot.State;
import frc.robot.State.DriveState;
import frc.robot.State.GrabHandState;
import frc.robot.State.MoveLeftAndRightArmState;
import frc.robot.State.RollerState;
import frc.robot.State.Hand.RotateState;
import frc.robot.subClass.Const;

public class DriveMode extends Mode {

    private static GrabGamePiecePhase phase = GrabGamePiecePhase.Phase1;

    @Override
    public void changeMode() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void changeState() {
        State.driveXSpeed = -driveController.getLeftY();
        State.driveZRotation = -driveController.getRightX();
        State.driveState = DriveState.s_fastDrive;

        //RT: intake, LT: outtake
        if(driveController.getRightTriggerAxis() > 0.5){
            State.intakeState = RollerState.s_intakeGamePiece;
        }else if(driveController.getLeftTriggerAxis() > 0.5){
            State.intakeState = RollerState.s_outtakeGamePiece;
        }else{
            State.intakeState = RollerState.s_stopRoller;
        }

        //YボタンでBasicPositionに戻る, XボタンでゲームピースをつかんでbasicPositionに戻る
        if(driveController.getYButton()){
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.basicPositionHeight;
            State.Arm.targetDepth = Const.Arm.basicPositionDepth;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.rotateState = RotateState.s_turnHandBack;
        }else if(driveController.getXButton()){
            switch(phase){
                case Phase1:
                State.grabHandState = GrabHandState.s_releaseHand;
                phase = GrabGamePiecePhase.Phase2;
                break;
                
                case Phase2:
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.Arm.targetHeight = Const.GrabGamePiecePhase.armIntakeHeight;
                State.Arm.targetDepth = Const.GrabGamePiecePhase.armIntakeDepth;
                if(State.Arm.isArmAtTarget){
                    phase = GrabGamePiecePhase.Phase3;
                }
                break;

                case Phase3:
                State.grabHandState = GrabHandState.s_grabHand;
                phase = GrabGamePiecePhase.Phase4;
                break;

                case Phase4:
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.Arm.targetHeight = -Const.GrabGamePiecePhase.armIntakeHeight;
                State.Arm.targetDepth = -Const.GrabGamePiecePhase.armIntakeDepth;
                break;
            }
        }

        if (driveController.getAButton()) {
            State.driveState = DriveState.s_apriltagTracking;
        } else if (driveController.getBButton()) {
            State.driveState = DriveState.s_targetTracking;
        }
    }
    
    enum GrabGamePiecePhase{
        //ハンドを開ける
        Phase1,
        //アームを下げる
        Phase2,
        //ハンドを閉める
        Phase3,
        //アームを上げる
        Phase4
    }
}
