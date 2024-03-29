package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.states.State;
import frc.robot.component.*;
import frc.robot.consts.Const;
import frc.robot.phase.Autonomous;
import frc.robot.subClass.ExternalSensors;
import frc.robot.subClass.Util;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;


public class Robot extends TimedRobot {

    ArrayList<Component> components;

    ExternalSensors externalSensors;

    // MQTT mqtt = new MQTT();

    PrintStream defaultConsole = System.out;
    ByteArrayOutputStream newConsole = new ByteArrayOutputStream();
    private NetworkTable table;
    NetworkTableEntry entry;

    @Override
    public void robotInit() {
        System.setOut(new PrintStream(newConsole));
        Const.ConstInit();

        // Thread thread = new Thread(() -> mqtt.connect());
        // thread.start();_

        components = new ArrayList<>();
        components.add(new Drive());
        components.add(new Intake());
        components.add(new Hand());
        components.add(new Arm());
        components.add(new Camera());
        components.add(new Limelight());

        externalSensors = new ExternalSensors();

        State.StateInit();

        // defaultConsole.print(newConsole);
        // newConsole = new ByteArrayOutputStream();
        table =  NetworkTableInstance.getDefault().getTable("SmartDashboard");
        entry = table.getEntry("center");
    }

    @Override
    public void robotPeriodic() {
        // defaultConsole.print(newConsole);
        // newConsole = new ByteArrayOutputStream();
        double[] array = entry.getDoubleArray(new double[]{0.0, 0.0});
        System.out.println("NetworkTables");
        System.out.println(array[0]);
        System.out.println(array[1]);
    }

    @Override
    public void autonomousInit() {
        for (Component component : components) {
            component.autonomousInit();
        }
        Autonomous.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        State.StateReset();
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }

        Autonomous.run();

        for (Component component : components) {
            component.applyState();
        }
        Util.allSendConsole();
    }

    @Override
    public void teleopInit() {
        State.mode = State.Modes.k_drive;

        for (Component component : components) {
            component.teleopInit();
        }
    }

    @Override
    public void teleopPeriodic() {
        State.StateReset();
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }

        State.mode.changeMode();

        State.mode.changeState();

        for (Component component : components) {
            component.applyState();
        }
        Util.allSendConsole();
    }

    @Override
    public void disabledInit() {
        for (Component component : components) {
            component.disabledInit();
        }
    }

    @Override
    public void disabledPeriodic() {
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }
        Util.allSendConsole();
    }

    @Override
    public void testInit() {
        State.mode = State.Modes.k_test;

        for (Component component : components) {
            component.testInit();
        }
    }

    @Override
    public void testPeriodic() {
        externalSensors.readExternalSensors();
        State.StateReset();
        for (Component component : components) {
            component.readSensors();
        }
        State.mode.changeState();

        for (Component component : components) {
           component.applyState();
        }
        Util.allSendConsole();
    }
}
