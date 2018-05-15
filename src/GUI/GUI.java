package GUI;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

public class GUI extends Application {
    @Override
    public void start(Stage stage) throws Exception {
        FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("sample.fxml"));
        fxmlLoader.setController(new Controller());
        Parent root = fxmlLoader.load();
        stage.setTitle("PID Controller: Car Speed Simulation");
        Scene scene = new Scene(root, 1096, 660);
        stage.setScene(scene);
        stage.setResizable(false);
        stage.setOnCloseRequest(windowEvent -> {
            Platform.exit();
            System.exit(0);
        });
        stage.show();
    }
}
