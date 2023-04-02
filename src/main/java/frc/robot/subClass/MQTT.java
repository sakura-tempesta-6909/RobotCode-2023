package frc.robot.subClass;

import frc.robot.States.State;
import frc.robot.consts.MQTTConst;
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
    private ConnectStatus connectStatus = ConnectStatus.notYet;
    private MqttConnectOptions connOpts = new MqttConnectOptions();
    int retryCount = 0;


    public void connect(){
        if (connectStatus == ConnectStatus.notYet) {
            try {
                mqttClient = new MqttClient(MQTTConst.Broker, MQTTConst.ClientId, new MemoryPersistence());
            } catch (MqttException e) {
                connectStatus = ConnectStatus.failedCreateClient;
                return;
            }
            connOpts.setCleanSession(false);
        }
        if(mqttClient.isConnected()) {
            return;
        }
        if (retryCount < MQTTConst.MaxRetry) {
            try {
                mqttClient.connect(connOpts);
                connectStatus = ConnectStatus.connected;
            } catch (MqttException e) {
                connectStatus = ConnectStatus.retryConnect;
            }
            retryCount++;
        }
    }
    public JSONObject convertStateToJson() {
        Map<String, Object> map = new HashMap<>();
        Field[] classes = State.class.getDeclaredFields();
        for (Field cl : classes) {
            try {
                map.put(cl.getName(), cl.get(State.class));
            } catch (IllegalAccessException e) {
            }
        }
        return new JSONObject(map);
    }
    public void publish(JSONObject json){
        if (connectStatus == ConnectStatus.connected) {
            try {
                mqttClient.publish(MQTTConst.Topic, new MqttMessage(json.toString().getBytes()));
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