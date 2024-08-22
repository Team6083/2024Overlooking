package frc.robot;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import java.net.URI;
import java.net.URISyntaxException;

public class RobotWebSocketClient extends WebSocketClient {

    public RobotWebSocketClient(URI serverUri) {
        super(serverUri);
    }

    @Override
    public void onOpen(ServerHandshake handshakedata) {
        System.out.println("Connected to server");
    }

    @Override
    public void onMessage(String message) {
        System.out.println("Message from server: " + message);
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
        System.out.println("Disconnected from server");
    }

    @Override
    public void onError(Exception ex) {
        ex.printStackTrace();
    }

    public static void main(String[] args) {
        try {
            // Replace 'localhost' with the IP address of the server if necessary
            URI serverUri = new URI("ws://localhost:3000/new_ui.html");
            RobotWebSocketClient client = new RobotWebSocketClient(serverUri);
            client.connect();

            // Example of sending data
            client.send("{\"status\": \"Active\"}");

        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
    }
}
