package PIDController;

import CarSimulator.CarSimulator;
import GUI.Observer.Subject;
import GUI.Observer.SubjectState;

import java.util.ArrayList;

/**
 * Wrapper class for the PID controller
 *
 * @author Nicolas Dutly
 * @version 1.0
 */
public class PIDController extends Subject implements Runnable {

    /**
     * the current PID parameters
     */
    private PIDParams pidParams;
    /**
     * The optimal PID parameters,
     * reverted to upon user request (reset button)
     */
    private PIDParams pidParams_optimal;
    /**
     * The controller's setpoint
     */
    private double sp;
    /**
     * The car's current speed
     */
    private double currentSpeed;
    /**
     * The current speed deltaa
     */
    private double currentError;
    /**
     * boolean specifying whether
     * to limit the car's acceleration
     */
    private boolean limAcc = true;

    /**
     * Create a new PID controller with the specified
     * PID parameters and a initial setpoint
     * @param pidParams the PID parameters
     * @param sp the initial setpoint
     */
    public PIDController(PIDParams pidParams, double sp) {
        this.pidParams = pidParams;
        this.pidParams_optimal = new PIDParams(pidParams.getKP(), pidParams.getKD(), pidParams.getKI());
        this.sp = sp;
    }

    @Override
    public void run() {
        double s_error;
        double s_old_error = 0;
        double integral = 0;
        double derivative;
        double val;
        CarSimulator simulator = new CarSimulator();
        new Thread(simulator).start();
        long nanoTime_old = System.nanoTime();
        ArrayList<Double> integral_vals = new ArrayList<>();
        while (true) {
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (integral_vals.size() == 20) integral_vals.clear();
            long nanoTime = System.nanoTime();
            double iteration_time = (nanoTime - nanoTime_old) / 1E9;
            s_error = sp - simulator.getSpeed();
            integral_vals.add(s_error * iteration_time);
            integral = integral_vals.stream()
                    .mapToDouble(a -> a)
                    .sum();
            derivative = (s_error - s_old_error) / iteration_time;
            //val = 3.9074321196717205 * error + 0.0061901094526276845 * derivative + 0.009132475994355075 * integral;
            val = pidParams.getKP() * s_error + pidParams.getKD() * derivative + pidParams.getKI() * integral;
            if (limAcc) {
                if (val > 6) val = 6;
                if (val < -6) val = -6;
            }
            simulator.setAcceleration(val);
            this.currentError = s_error;
            this.currentSpeed = simulator.getSpeed();
            s_old_error = s_error;
            nanoTime_old = System.nanoTime();
            super.notify_observers();
        }
    }

    /**
     * Reset the PID parameters
     */
    public void reset_params() {
        this.pidParams = this.pidParams_optimal;
    }

    /**
     * Get the simulator's current speed
     * @return the current speed
     */
    public double getCurrentSpeed() {
        return currentSpeed;
    }


    /**
     * Set the simulator's current set point
     */
    public void setSp(double sp) {
        this.sp = sp;
    }

    /**
     * Get the simulator's current PID parameters
     * @return the current PID parameters
     */
    public PIDParams getPidParams() {
        return pidParams;
    }

    /**
     * Specify new PID parameters
     * @param pidParams the new PID parameters
     */
    public void setPidParams(PIDParams pidParams) {
        this.pidParams = pidParams;
    }

    /**
     * Specify whether to limit the car's acceleration
     * @param limAcc value
     */
    public void setLimAcc(boolean limAcc) {
        this.limAcc = limAcc;
    }

    public SubjectState getState(){
        return new SubjectState(this.currentSpeed, this.currentError, this.sp, this.pidParams);
    }

}
