import CarSimulator.CarSimulator;
public class Main {
    public static void main(String[] args) {
        double kp = 0.3;
        int kd = 0;
        int ki = 0;
        int sp = 20;
        double error;
        double old_error = 0;
        double integral = 0;
        double derivative = 0;
        double dt = 1;
        double val;
        CarSimulator simulator = new CarSimulator();
        new Thread(simulator).start();
        System.out.println(simulator.getSpeed());
        while(true){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            error = sp - simulator.getSpeed();
            integral += error;
            derivative = (error-old_error) / dt;
            val = kp * error + kd * derivative + ki * integral;
            simulator.setAcceleration(val);
            old_error = error;
            System.out.printf("error: %f\n", error);
        }

    }
}
