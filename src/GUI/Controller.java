package GUI;

import PIDController.PIDController;
import PIDController.PIDParams;
import ParticleSwarmOptimizer.ParticleSwarmOptimizer;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.*;

import java.net.URL;
import java.util.ResourceBundle;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Controller implements Initializable {

    private static final int MAX_DATA_POINTS = 50;
    private int xSeriesData = 0;
    private XYChart.Series<Number, Number> series1 = new XYChart.Series<>();
    private XYChart.Series<Number, Number> series2 = new XYChart.Series<>();
    private XYChart.Series<Number, Number> series3 = new XYChart.Series<>();
    private ExecutorService executor;
    private ConcurrentLinkedQueue<Number> dataQ1 = new ConcurrentLinkedQueue<>();
    private ConcurrentLinkedQueue<Number> dataQ2 = new ConcurrentLinkedQueue<>();
    private ConcurrentLinkedQueue<Number> dataQ3 = new ConcurrentLinkedQueue<>();

    private PIDController pidController;


    @FXML
    private LineChart<Number, Number> pid_chart;

    @FXML
    private NumberAxis y_axis;

    @FXML
    private NumberAxis x_axis;

    @FXML
    private Slider sp_slider;

    @FXML
    private CheckBox adv_ctrl;

    @FXML
    private TextField kp_user_in;

    @FXML
    private CheckBox acc_lim_toggle;

    @FXML
    private TextField ki_user_in;

    @FXML
    private TextField kd_user_in;

    @FXML
    private Button reset_btn;

    @FXML
    private Button start_pso;

    @FXML
    private Button stop_pso;

    @FXML
    private TextArea console0;

    public void updateConsole(String str) {
        console0.setText(console0.getText() + str);
    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        pidController = new PIDController(new PIDParams(5.5958884831108175, 0.18911251211841631, 0.001709275613215766), 20);
        pidController.setSp(sp_slider.getValue());
        new Thread(pidController).start();
        pid_chart.getData().addAll(series1, series2, series3);
        executor = Executors.newCachedThreadPool(r -> {
            Thread thread = new Thread(r);
            thread.setDaemon(true);
            return thread;
        });

        AddToQueue addToQueue = new AddToQueue();
        executor.execute(addToQueue);
        prepareTimeline();
        series1.setName("Desired speed");
        series2.setName("Actual speed");
        series3.setName("Error");
        sp_slider.valueProperty().addListener((observableValue, number, t1) -> {
            pidController.setSp(sp_slider.getValue());
        });
        kp_user_in.setText(String.valueOf(pidController.getPidParams().getKP()));
        kd_user_in.setText(String.valueOf(pidController.getPidParams().getKD()));
        ki_user_in.setText(String.valueOf(pidController.getPidParams().getKI()));
        acc_lim_toggle.setSelected(true);
        if (!adv_ctrl.isSelected()) {
            kp_user_in.setDisable(true);
            kd_user_in.setDisable(true);
            ki_user_in.setDisable(true);
            acc_lim_toggle.setDisable(true);
            reset_btn.setDisable(true);
        }
        adv_ctrl.selectedProperty().addListener((observableValue, aBoolean, t1) -> {
            if (!adv_ctrl.isSelected()) {
                kp_user_in.setDisable(true);
                kd_user_in.setDisable(true);
                ki_user_in.setDisable(true);
                acc_lim_toggle.setDisable(true);
                reset_btn.setDisable(true);
            }else{
                kp_user_in.setDisable(false);
                kd_user_in.setDisable(false);
                ki_user_in.setDisable(false);
                acc_lim_toggle.setDisable(false);
                reset_btn.setDisable(false);
            }
        });
        kp_user_in.textProperty().addListener((observableValue, s, t1) -> {
            pidController.setPidParams(new PIDParams(Double.valueOf(kp_user_in.getText()),
                    pidController.getPidParams().getKD(),
                    pidController.getPidParams().getKI()));
        });
        kd_user_in.textProperty().addListener((observableValue, s, t1) -> {
            pidController.setPidParams(new PIDParams(pidController.getPidParams().getKP(),
                    Double.valueOf(kd_user_in.getText()),
                    pidController.getPidParams().getKI()));
        });
        ki_user_in.textProperty().addListener((observableValue, s, t1) -> {
            pidController.setPidParams(new PIDParams(pidController.getPidParams().getKP(),
                    pidController.getPidParams().getKD(),
                    Double.valueOf(ki_user_in.getText())));
        });
        acc_lim_toggle.selectedProperty().addListener((observableValue, aBoolean, t1) ->
                pidController.setLimAcc(acc_lim_toggle.isSelected())
        );

        reset_btn.setOnAction(actionEvent -> {
            pidController.reset_params();
            kp_user_in.setText(String.valueOf(pidController.getPidParams().getKP()));
            kd_user_in.setText(String.valueOf(pidController.getPidParams().getKD()));
            ki_user_in.setText(String.valueOf(pidController.getPidParams().getKI()));
        });

        stop_pso.setDisable(true);

        start_pso.setOnAction(actionEvent -> {
            Alert alert = new Alert(Alert.AlertType.WARNING,
                    "",
                    ButtonType.YES,
                    ButtonType.NO,
                    ButtonType.CANCEL);
            Label label = new Label("The PSO algorithm is multi-threaded and will only work\nwell on a multicore (>5) machine. Your machine may become unresponsive. Continue?");
            label.setWrapText(true);
            alert.getDialogPane().setContent(label);
            alert.showAndWait();

            if (alert.getResult() == ButtonType.YES) {
                ParticleSwarmOptimizer pso = new ParticleSwarmOptimizer(10);
                new Thread(pso).start();
                new Thread(() -> {
                    PIDParams oldOptimum = new PIDParams(0,0,0);
                    while(pso.isContinue_condition()){
                        PIDParams swarmOptimum = pso.getSwarmOptimum();
                        pidController.setPidParams(pso.getSwarmOptimum());
                        if (!swarmOptimum.equals(oldOptimum)) {
                            updateConsole("new swam optimum found: " + swarmOptimum.toString() + "\n");
                            oldOptimum = swarmOptimum;
                        }
                    }

                }).start();
            }
        });

    }

    private void prepareTimeline() {
        new AnimationTimer() {
            @Override
            public void handle(long now) {
                addDataToSeries();
            }
        }.start();
    }

    private void addDataToSeries() {
        for (int i = 0; i < 20; i++) { //-- add 20 numbers to the plot+
            if (dataQ1.isEmpty()) break;
            series1.getData().add(new XYChart.Data<>(xSeriesData++, dataQ1.remove()));
            series2.getData().add(new XYChart.Data<>(xSeriesData++, dataQ2.remove()));
            series3.getData().add(new XYChart.Data<>(xSeriesData++, dataQ3.remove()));
        }
        // remove points to keep us at no more than MAX_DATA_POINTS
        if (series1.getData().size() > MAX_DATA_POINTS) {
            series1.getData().remove(0, series1.getData().size() - MAX_DATA_POINTS);
        }
        if (series2.getData().size() > MAX_DATA_POINTS) {
            series2.getData().remove(0, series2.getData().size() - MAX_DATA_POINTS);
        }
        if (series3.getData().size() > MAX_DATA_POINTS) {
            series3.getData().remove(0, series3.getData().size() - MAX_DATA_POINTS);
        }
        // update
        x_axis.setLowerBound(xSeriesData - MAX_DATA_POINTS);
        x_axis.setUpperBound(xSeriesData - 1);
    }


    private class AddToQueue implements Runnable {
        public void run() {
            try {
                // add a item of random data to queue
                dataQ1.add(pidController.getSp());
                dataQ2.add(pidController.getCurrentSpeed());
                dataQ3.add(pidController.getCurrentError());
                Thread.sleep(50);
                executor.execute(this);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
}
