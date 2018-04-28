package GUI;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.util.concurrent.Executors;

public class GUI extends Application {
    @Override
    public void start(Stage stage) throws Exception {
        FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("sample.fxml"));
        fxmlLoader.setController(new Controller());
        Parent root = fxmlLoader.load();
        stage.setTitle("PID Controller: Car Speed Simulation");
        stage.setScene(new Scene(root, 1096, 660));
        stage.show();
    }
}
