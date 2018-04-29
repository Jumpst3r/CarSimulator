package PIDController;

import CarSimulator.CarSimulator;
import ParticleSwarmOptimizer.AbsErr;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class PIDController implements Runnable{
    private PIDParams pidParams;
    private PIDParams pidParams_optimal;
    private double sp;
    private double currentSpeed;
    private double currentError;
    private boolean limAcc = true;


    public PIDController(PIDParams pidParams, double sp) {
        this.pidParams = pidParams;
        this.pidParams_optimal = new PIDParams(pidParams.getKP(), pidParams.getKD(), pidParams.getKI());
        this.sp = sp;
    }

    @Override
    public void run() {
        double error;
        double old_error = 0;
        double integral = 0;
        double derivative;
        double val;
        CarSimulator simulator = new CarSimulator();
        new Thread(simulator).start();
        long nanoTime_old = System.nanoTime();
        AbsErr absErr = new AbsErr(0);
        long k = 0;
        while (true) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            long nanoTime = System.nanoTime();
            double iteration_time = (nanoTime - nanoTime_old)/1E9;
            error = sp - simulator.getSpeed();
            //System.out.println("mse: " + mse);
            integral = integral + (error * iteration_time);
            //System.out.println("integral" + integral);
            derivative = (error - old_error) / iteration_time;
            //val = 3.9074321196717205 * error + 0.0061901094526276845 * derivative + 0.009132475994355075 * integral;
            val = pidParams.getKP() * error + pidParams.getKD() * derivative + pidParams.getKI() * integral;
            if (limAcc) {
                if (val > 6) val = 6;
                if (val < -6) val = -6;
            }
            simulator.setAcceleration(val);
            this.currentError = error;
            this.currentSpeed = simulator.getSpeed();
            old_error = error;
            nanoTime_old = System.nanoTime();
            k++;
        }
    }

    public void reset_params() {
        this.pidParams = this.pidParams_optimal;
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    public double getCurrentError() {
        return currentError;
    }

    public double getSp() {
        return sp;
    }

    public void setSp(double sp) {
        this.sp = sp;
    }

    public PIDParams getPidParams() {
        return pidParams;
    }

    public void setPidParams(PIDParams pidParams) {
        this.pidParams = pidParams;
    }

    public void setLimAcc(boolean limAcc) {
        this.limAcc = limAcc;
    }
}
