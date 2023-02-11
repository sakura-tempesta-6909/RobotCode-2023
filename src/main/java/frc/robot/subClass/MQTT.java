package frc.robot.subClass;

import frc.robot.State;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.json.JSONObject;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public class MQTT{
    private MqttClient mqttClient;
    final String broker = "tcp://raspberrypi.local:1883";
    final String topic = "robot/data/main";
    final int qos = 1;
    final String clientId = "robot/test";
    ConnectStatus connectStatus = ConnectStatus.notYet;
    MqttConnectOptions connOpts = new MqttConnectOptions();
    int retryCount = 0;


    public void connect(){
        if (connectStatus == ConnectStatus.notYet) {
            try {
                mqttClient = new MqttClient(broker, clientId, new MemoryPersistence());
            } catch (MqttException e) {
                connectStatus = ConnectStatus.failedCreateClient;
                return;
            }
            connOpts.setCleanSession(false);
        }
        if(mqttClient.isConnected()) {
            return;
        }
        if (retryCount < 100) {
            try {
                mqttClient.connect(connOpts);
                connectStatus = ConnectStatus.connected;
            } catch (MqttException e) {
                retryCount++;
                connectStatus = ConnectStatus.retryConnect;
            }
        }
    }
    public JSONObject convertStateToJson() {
        Map<String, Object> map = new HashMap<>();
        Field[] classes = State.class.getDeclaredFields();
        for (Field cl : classes) {
            try {
                map.put(cl.getName(), cl.get(State.class));
            } catch (IllegalAccessException e) {
                continue;
            }
        }
        return new JSONObject(map);
    }
    public void publish(JSONObject json){
        if (connectStatus == ConnectStatus.connected) {
            try {
                mqttClient.publish(topic, new MqttMessage(json.toString().getBytes()));
            } catch (MqttException e) {
                System.out.println("MQTT Publish Error");
            }
        }
    }

    public void publishState(){
        this.publish(this.convertStateToJson());
    }

    enum ConnectStatus {
        notYet,
        failedCreateClient,
        retryConnect,
        failedConnect,
        connected,
        unknown,
    }
}
