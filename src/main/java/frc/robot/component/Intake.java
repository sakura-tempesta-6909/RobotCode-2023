package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.State;
import frc.robot.subClass.Const;

public class Intake implements Component{
    private Solenoid intakeSolenoid;
    private VictorSPX rightRoller;
    private VictorSPX leftRoller;
    private VictorSPX bottomRoller;

    public Intake() {
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Const.Ports.IntakeSolenoid);
        rightRoller = new VictorSPX(Const.Ports.RightRoller);
        leftRoller = new VictorSPX(Const.Ports.LeftRoller);
        bottomRoller = new VictorSPX(Const.Ports.BottomRoller);
    }
        
    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void readSensors() {
        // TODO Auto-generated method stub
        
    }

    /** Roller関係のモーターを動かす */
    // public void rollerControl(double sideRollerSpeed, double bottomRollerSpeed) {
    //     rightRoller.set(1, sideRollerSpeed);
    //     leftRoller.set(1, sideRollerSpeed);
    //     bottomRoller.set(1, bottomRollerSpeed);

    // }

    /** Intake関係のsolenoidを動かす */
    public void intakeControl(boolean isExtendingIntake) {
        intakeSolenoid.set(isExtendingIntake);
    }

    // /** CONEとCUBEを出す */
    // public void outtakeGamePiece() {
    //     rollerControl(Const.Speeds.SideRollerOuttakeSpeed);
    // }

    // /** CONEとCUBEを回収する */
    // public void intakeGamePiece() {
    //     rollerControl(Const.Speeds.SideRollerIntakeSpeed);
    // }

    // /** Conveyorを動かさない */
    // public void stopRoller() {
    //     rollerControl(Const.Speeds.Neutral);
    // }

    /** Intakeを出す */
    public void openIntake() {
        intakeControl(true);
    }

    /** Rollerをしまう */
    public void closeIntake() {
        intakeControl(false);
    }

    @Override
    public void applyState() {
        switch(State.intakeState){
            // case s_outtakeGamePiece:
            //     outtakeGamePiece();
            //     break;
            // case s_intakeGamePiece:
            //     intakeGamePiece();
            //     break;
            // case s_stopRoller:
            //     stopRoller();
            //     break;
        }
        
        switch(State.intakeExtensionState){
            case s_openIntake:
                openIntake();
                break;
            case s_closeIntake:
                closeIntake();
                break;
        }
    }
}