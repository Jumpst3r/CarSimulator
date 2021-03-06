package GUI;

import GUI.Observer.Observer;
import GUI.Observer.Subject;
import GUI.Observer.SubjectState;
import PIDController.PIDController;
import PIDController.PIDParams;
import ParticleSwarmOptimizer.ParticleSwarmOptimizer;
import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.concurrent.Task;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.*;

import java.io.IOException;
import java.io.OutputStream;
import java.net.URL;
import java.util.Random;
import java.util.ResourceBundle;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadLocalRandom;

public class Controller extends Observer implements Initializable {
    /**
     * This class regroups all UI elements. The
     * code for the javafx linechart element is
     * taken from https://stackoverflow.com/questions/22089022/line-chart-live-update,
     * the post by user ItachiUchiha which consists of
     * the functions addDataToSeries() addDataToSeries() prepareTimeline()
     */
    private static final int MAX_DATA_POINTS = 600;
    private int xSeriesData = 0;
    private XYChart.Series<Number, Number> series1 = new XYChart.Series<>();
    private XYChart.Series<Number, Number> series2 = new XYChart.Series<>();
    private XYChart.Series<Number, Number> series3 = new XYChart.Series<>();
    private ExecutorService executor;
    private ConcurrentLinkedQueue<Number> dataQ1 = new ConcurrentLinkedQueue<>();
    private ConcurrentLinkedQueue<Number> dataQ2 = new ConcurrentLinkedQueue<>();
    private ConcurrentLinkedQueue<Number> dataQ3 = new ConcurrentLinkedQueue<>();
    public static Console CONSOLE;

    private PIDController pidController;
    private ParticleSwarmOptimizer pso;
    private Subject pidSubject;


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

    @FXML
    private CheckBox rnd_speed;

    @FXML
    private Label speed_lbl;

    @FXML
    private CheckBox multi_threaded_chbox;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        CONSOLE = new Console(console0);
        pidController = new PIDController(new PIDParams(5.5958884831108175, 0.18911251211841631, 0.001709275613215766), 20);
        pidController.setSp(sp_slider.getValue());
        pidSubject = pidController;
        pidSubject.register(this);
        new Thread(pidController).start();
        pid_chart.getData().addAll(series1, series2, series3);
        executor = Executors.newCachedThreadPool(r -> {
            Thread thread = new Thread(r);
            thread.setDaemon(true);
            return thread;
        });
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

        multi_threaded_chbox.selectedProperty().addListener((observable, oldValue, newValue) -> {
            if (multi_threaded_chbox.isSelected()) {
                Alert alert = new Alert(Alert.AlertType.WARNING,
                        "",
                        ButtonType.YES,
                        ButtonType.NO);
                Label label = new Label("Enabling multithreading will run the algorithm much faster but will severely\nslow done the UI and your computer may become unresponsive.");
                alert.setContentText("Enable Multi-threading?");
                alert.setTitle("Enable Multi-threading?");
                label.setWrapText(true);
                alert.getDialogPane().setContent(label);
                alert.showAndWait();
                if (alert.getResult() == ButtonType.YES) {
                    multi_threaded_chbox.setSelected(true);
                }
                if (alert.getResult() == ButtonType.NO) {
                    multi_threaded_chbox.setSelected(false);
                }
            }
        });

        reset_btn.setOnAction(actionEvent -> {
            pidController.reset_params();
            kp_user_in.setText(String.valueOf(pidController.getPidParams().getKP()));
            kd_user_in.setText(String.valueOf(pidController.getPidParams().getKD()));
            ki_user_in.setText(String.valueOf(pidController.getPidParams().getKI()));
        });
        Runnable a = new speedRandomizerThread();
        Thread speed_randomizer = new Thread(a);
        stop_pso.setDisable(true);
        start_pso.setOnAction(actionEvent -> {
            Alert alert = new Alert(Alert.AlertType.INFORMATION,
                    "",
                    ButtonType.YES,
                    ButtonType.NO,
                    ButtonType.CANCEL);
            Label label = new Label("The PSO algorithm will now try to find optimal parameters. After each generation,\n" +
                    "the optimal solutions will be applied to the PID controller to visualize progress.\n Due to the heuristic" +
                    " nature of the algorithm, it is not guaranteed that an optimal solution will be found.\n In that case restart" +
                    " the algorithm. Proceed?");
            label.setWrapText(true);
            alert.getDialogPane().setContent(label);
            alert.showAndWait();
            if (alert.getResult() == ButtonType.YES) {
                rnd_speed.setSelected(true);
                pso = new ParticleSwarmOptimizer(10);
                pso.setMulti_threaded(multi_threaded_chbox.isSelected());
                Task pso_task = new Task() {
                    @Override
                    protected Object call() throws Exception {
                        pso.run();
                        return null;
                    }
                };
                new Thread(pso_task).start();
                new Thread(() -> {
                    start_pso.setDisable(true);
                    stop_pso.setDisable(false);
                    PIDParams old_params = new PIDParams(0, 0, 0);
                    while(pso.isContinue_condition()){
                        pidController.setPidParams(pso.getSwarmOptimum());
                        if (!old_params.equals_pid(pso.getSwarmOptimum())) {
                            kp_user_in.setText(String.valueOf(pso.getSwarmOptimum().getKP()));
                            kd_user_in.setText(String.valueOf(String.valueOf(pso.getSwarmOptimum().getKD())));
                            ki_user_in.setText(String.valueOf(String.valueOf(pso.getSwarmOptimum().getKI())));
                            old_params = new PIDParams(pso.getSwarmOptimum().getKP(), pso.getSwarmOptimum().getKD(), pso.getSwarmOptimum().getKI());
                        }
                    }

                }).start();
            }
        });
        stop_pso.setOnAction(actionEvent -> {
            pso.setContinue_condition(false);
            stop_pso.setDisable(true);
            start_pso.setDisable(false);
            rnd_speed.setSelected(false);
        });
        rnd_speed.selectedProperty().addListener((observableValue, number, t1)->{
            if (rnd_speed.isSelected()) {
                sp_slider.setDisable(true);
                speed_randomizer.start();
            }
            if (!rnd_speed.isSelected()) {
                sp_slider.setDisable(false);
                speed_randomizer.interrupt();
            }
        });



    }

    //code from so:
    private void prepareTimeline() {
        new AnimationTimer() {
            @Override
            public void handle(long now) {
                addDataToSeries();
                speed_lbl.setText("" + (int) pidController.getCurrentSpeed() + "m/s");
            }
        }.start();
    }

    private void addDataToSeries() {
        for (int i = 0; i < 20; i++) {
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
        x_axis.setLowerBound(xSeriesData - MAX_DATA_POINTS);
        x_axis.setUpperBound(xSeriesData - 1);
    }

    @Override
    public void update() {
        SubjectState state = pidSubject.getState();
        dataQ1.add(state.getSetPoint());
        dataQ2.add(state.getCurrentSpeed());
        dataQ3.add(state.getCurrentError());
    }

    //end code from so
    public static class Console extends OutputStream {

        private TextArea output;

        Console(TextArea textArea) {
            this.output = textArea;
        }

        @Override
        public void write(int i) throws IOException {
            Platform.runLater(()-> output.appendText(String.valueOf((char) i)));
        }

        public void write(String str, String lvl) throws IOException {

            if (lvl.compareTo("info") == 0) str = "[INFO]\t" + str;
            if (lvl.compareTo("warning") == 0) str = "[WARNING]\t" + str;
            for (char c: str.toCharArray()) {
                write(c);
            }
        }

        public void write(String str) throws IOException {
            for (char c: str.toCharArray()) {
                write(c);
            }
        }
    }

    private class speedRandomizerThread implements Runnable{

        private Thread thread;

        speedRandomizerThread() {
            if (thread == null) {
                thread = new Thread(() -> {
                    while (true) {
                        if (rnd_speed.isSelected()) {
                            Random rnd = new Random();
                            try {
                                Thread.sleep((long) (400 + rnd.nextDouble() * 4000));
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            int randomNum = ThreadLocalRandom.current().nextInt(0, 50);
                            pidController.setSp(randomNum);
                            sp_slider.setValue(randomNum);
                        }
                    }
                });
            }
        }

        @Override
        public void run() {
            thread.start();
        }
        }
}
