package ParticleSwarmOptimizer;

import CarSimulator.CarSimulator;
import PIDController.PIDParams;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * This class represents a particle in the particle swarm.
 * Each particle has a location in the 3dim search space,
 * given by its PID parameters and a best known local
 * position.
 *
 * @author Nicolas Dutly
 * @version 1.0
 */
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

    /**
     * Creates a new Particle
     * @param pso the pso in which the particle lives
     */
    public Particle(ParticleSwarmOptimizer pso) {
        //assign random positions and velocities for each particle
        double kp_init = pso.unifgen(pso.getLowerBound(), pso.getUpperBound());
        double kd_init = pso.unifgen(pso.getLowerBound(), pso.getUpperBound());
        double ki_init = pso.unifgen(pso.getLowerBound(), pso.getUpperBound());
        for (int i = 0; i < 3; i++) {
            this.velocity[i] = pso.unifgen(Math.abs(-pso.getUpperBound() - pso.getLowerBound()), Math.abs(pso.getUpperBound() - pso.getLowerBound()));
        }
        this.location = new PIDParams(kp_init, kd_init, ki_init);
        this.bestKnownParams = this.location;
        this.pso = pso;
    }

    /**
     * Evaluates the particle's score by running a simulation for
     * a few seconds and calculating the MSE
     */
    synchronized void eval() {
        CountDownLatch countDownLatch = new CountDownLatch(1);
        //wrapper class for the error:
        AbsErr absErr = new AbsErr(0);
        new Thread(() -> {
            double s_error;
            double s_old_error = 0;
            double integral = 0;
            double derivative;
            double val;
            int sp = 10;
            CarSimulator simulator = new CarSimulator();
            new Thread(simulator).start();
            double mse = 0;
            long nanoTime_old = System.nanoTime();
            ArrayList<Double> integral_vals = new ArrayList<>();
            int k = 0;
            long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(10);
            while (stop > System.nanoTime()) {
                try {
                    Thread.sleep(25);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                //vary the sp slightly during simulation
                if (k++ == 50) sp += 3;
                if (k == 60) sp += 3;
                long nanoTime = System.nanoTime();
                double dt = (nanoTime - nanoTime_old)/1E9;
                //only use the last 20 values for the integral
                if (integral_vals.size() == 20) integral_vals.clear();
                s_error = sp - simulator.getSpeed();
                mse += Math.pow(s_error, 2);
                integral_vals.add(s_error * dt);
                integral = integral_vals.stream()
                        .mapToDouble(a -> a)
                        .sum();
                derivative = (s_error - s_old_error) / dt;
                val = location.getKP() * s_error + location.getKD() * derivative + location.getKI() * integral;
                //it's better to train without acceleration limits
                /*if (val > 6) val = 6;
                if (val < -6) val = -6;*/
                simulator.setAcceleration(val);
                s_old_error = s_error;
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
        //update personal best known parameters
        if (absErr.getAbsErr() < absErr.getAbsErr()){
            this.bestKnownParams = this.location;
            this.bestKnownParams.setValue(absErr.getAbsErr());
        }
        eval_score = absErr.getAbsErr();
        location.setValue(eval_score);
        /*
          the updateSwarmOptimum function checks if the particle's
          is better than the swarm's best known position and updates
          it accordingly.
         */
        pso.updateSwarmOptimum(location);
    }

    /**
     * Updates the particle's position according the the pso formula.
     * The particle's position are the PID parameters
     */
    void updatePosition() {
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
}
