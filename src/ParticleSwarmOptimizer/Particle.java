package ParticleSwarmOptimizer;

import PIDController.PIDParams;

import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import CarSimulator.CarSimulator;

public class Particle {
    /**
     * Describes the particles best known
     * (local) parameters
     */
    private PIDParams bestKnownParams;

    /**
     * the particles current position in
     * the parameter space
     */
    private PIDParams location;

    /**
     * We want to minimize this variable,
     * which represents the MSE during a
     * 4 second simulation.
     */
    private double eval_score = Double.POSITIVE_INFINITY;
    /**
     * describes the particles velocity in each
     * parameter direction
     */
    private double[] velocity = new double[3];

    /**
     * points to the particle's pso
     */
    private ParticleSwarmOptimizer pso;

    private ArrayList<String> swarmStats = new ArrayList<>();
    private int n = 0;

    public Particle(ParticleSwarmOptimizer pso) {
        double kp_init = pso.unifgen(-pso.getUpperBound(), pso.getUpperBound());
        double kd_init = pso.unifgen(-pso.getUpperBound(), pso.getUpperBound());
        double ki_init = pso.unifgen(-pso.getUpperBound(), pso.getUpperBound());

        this.location = new PIDParams(kp_init, kd_init, ki_init);
        this.bestKnownParams = this.location;
        this.pso = pso;
    }

    public synchronized void eval() {
        CountDownLatch countDownLatch = new CountDownLatch(1);
        //Calculate old error using old kp
        AbsErr absErr = new AbsErr(0);
        new Thread(() -> {
            double error;
            double old_error = 0;
            double integral = 0;
            double derivative;
            double val;
            int sp = 20;
            CarSimulator simulator = new CarSimulator();
            new Thread(simulator).start();
            double mse = 0;
            long nanoTime_old = System.nanoTime();
            long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(10);
            int n = 0;
            while (stop > System.nanoTime()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                long nanoTime = System.nanoTime();
                double dt = (nanoTime - nanoTime_old)/1E9 + 100;
                error = sp - simulator.getSpeed();
                //System.out.println("error: " + error);
                mse += Math.pow(error, 2);
                //System.out.println("mse: " + mse);
                integral = integral + (error * dt);
                //System.out.println("integral" + integral);
                derivative = (error - old_error) / dt;
                //System.out.println("derivative " + derivative);
                val = location.getKP() * error + location.getKD() * derivative + location.getKI() * integral;
                //System.out.println("val: " + val);
                if (val > 6) val = 6;
                if (val < -6) val = -6;
                simulator.setAcceleration(val);
                old_error = error;
                n++;
                nanoTime_old = System.nanoTime();
            }
            absErr.setAbsErr(mse);
            simulator.stop();
            countDownLatch.countDown();
        }).start();
        try {
            countDownLatch.await();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (absErr.getAbsErr() < absErr.getAbsErr()){
            this.bestKnownParams = this.location;
            this.bestKnownParams.setValue(absErr.getAbsErr());
        }
        eval_score = absErr.getAbsErr();
        location.setValue(eval_score);
        pso.updateSwarmOptimum(location);

        swarmStats.add(n + "," + location.getKP() + "," + location.getKD() + "," + location.getKI() + "\n");
        n++;
    }

    public void updatePosition() {
        double new_pos[] = new double[3];
        for (int i = 0; i < 3; i++) {
            double rp = pso.unifgen(0, 1);
            double rg = pso.unifgen(0, 1);
            this.velocity[i] = ParticleSwarmOptimizer.INERTIA * this.velocity[i]
                    + ParticleSwarmOptimizer.W_LOCAL * rp * (bestKnownParams.getParams()[i] - location.getParams()[i])
                    + ParticleSwarmOptimizer.W_GLOBAL * rg * (pso.getSwarmOptimum().getParams()[i] - location.getParams()[i]);
            //if position is invalid (<0) move back into valid space
            new_pos[i] = Math.abs(this.location.getParams()[i] + this.velocity[i]);
        }
        this.location.setParams(new_pos);
    }

    public PIDParams getLocation() {
        return location;
    }

    public PIDParams getBestKnownParams() {
        return bestKnownParams;
    }

    public ArrayList<String> getSwarmStats() {
        return swarmStats;
    }
}
