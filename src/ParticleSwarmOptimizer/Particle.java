package ParticleSwarmOptimizer;

import PIDController.PIDParams;

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

    public Particle(ParticleSwarmOptimizer pso) {
        double kp_init = pso.unifgen(pso.getLowerBound(), pso.getUpperBound());
        double kd_init = pso.unifgen(pso.getLowerBound(), pso.getUpperBound());
        double ki_init = pso.unifgen(pso.getLowerBound(), pso.getUpperBound());
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
            double dt = 0.02;
            double val;
            int sp = 50;
            CarSimulator simulator = new CarSimulator();
            new Thread(simulator).start();
            simulator.setAcceleration(10);
            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(4);
            double mse = 0;
            while (stop > System.nanoTime()) {
                try {
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
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
                simulator.setAcceleration(val > 10 ? 10 : val);
                old_error = error;
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
        if (absErr.getAbsErr() < eval_score){
            this.bestKnownParams = this.location;
            this.bestKnownParams.setValue(absErr.getAbsErr());
        }
        eval_score = absErr.getAbsErr();
        pso.updateSwarmOptimum(location);
    }

    public void updatePosition() {
        double rp = pso.unifgen(0, 1);
        double rg = pso.unifgen(0, 1);
        for (int i = 0; i < 3; i++) {
            this.velocity[i] = ParticleSwarmOptimizer.INERTIA * this.velocity[i]
                    + ParticleSwarmOptimizer.W_LOCAL * rp * (bestKnownParams.getParams()[i] - location.getParams()[i])
                    + ParticleSwarmOptimizer.W_GLOBAL * rg * (pso.getSwarmOptimum().getParams()[i] - location.getParams()[i]);
            this.location.getParams()[i] = this.location.getParams()[i] + this.velocity[i];
        }
    }

    public PIDParams getBestKnownParams() {
        return bestKnownParams;
    }
}
