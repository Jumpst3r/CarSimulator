package ParticleSwarmOptimizer;

import GUI.Controller;
import PIDController.PIDParams;

import java.io.IOException;
import java.util.Random;
import java.util.concurrent.CountDownLatch;

/**
 * This class implements the particle swarm optimizer
 *
 * @author Nicolas Dutly
 * @version 1.0
 */
public class ParticleSwarmOptimizer implements Runnable{


    /**
     * the number of particles in the swarm
     */
    private int nbOfParticles;
    /**
     * the upper bound for the parameter space.
     * (ie assume K_p,d,i smaller or equal 10)
     */
    private final int upperBound = 2;

    /**
     * The lower bound for the parameter space
     * (ie assume K_p,d,i bigger or equal 0)
     */
    private final int lowerBound = 0;
    /**
     * represents the swarm's best known parameters
     */
    private volatile PIDParams swarmOptimum;

    /**
     * describes how strongly a particle moves towards
     * its best known position
     */
    static final double W_LOCAL = 2;

    /**
     * describes how strongly a particles moves towards
     * the swarm's optimum
     */
    static final double W_GLOBAL = 2;

    /**
     * describes the particles inertia
     */
    static final double INERTIA = 0.729;

    private boolean continue_condition = true;

    private Particle[] particles;

    private boolean multi_threaded = false;

    public ParticleSwarmOptimizer(int nbOfParticles) {
        this.nbOfParticles = nbOfParticles;
        this.particles = new Particle[nbOfParticles];
        this.swarmOptimum = new PIDParams(Double.POSITIVE_INFINITY);
        try {
            Controller.CONSOLE.write("====================================================\n");
            Controller.CONSOLE.write("starting swarm optimization with following params: \n");
            Controller.CONSOLE.write("\tnbOfParticles:\t\t\t\t10 \n");
            Controller.CONSOLE.write("\tparamter search space:\t\t["+lowerBound+","+upperBound+"]³ \n");
            Controller.CONSOLE.write("\tparticle inertia:\t\t\t"+INERTIA+" \n");
            Controller.CONSOLE.write("\tsocial weight:\t\t\t\t"+W_GLOBAL+" \n");
            Controller.CONSOLE.write("\tcognitive weight:\t\t\t"+W_LOCAL+" \n");
            Controller.CONSOLE.write("====================================================\n");
            Thread.sleep(200);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
        initParticles();
    }

    private void initParticles() {
        for (int i = 0; i < this.nbOfParticles; i++) {
            particles[i] = new Particle(this);
            try {
                Controller.CONSOLE.write("initialized particle " + i + "\n","info");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Returns a uniformly distributed number in
     * [lo,hi]
     *
     * @param lo lower bound
     * @param hi upper bound
     * @return X~Unif[lo,hi]
     */
    double unifgen(int lo, int hi) {
        Random r = new Random();
        return lo + (hi - lo) * r.nextDouble();
    }

    /**
     * Return's the PSO upper bound
     * @return the upper bound
     */
    int getUpperBound() {
        return upperBound;
    }

    /**
     * Return's the PSO lower bound
     * @return the lower bound
     */
    int getLowerBound() {
        return lowerBound;
    }

    /**
     * Return's the pso optimum
     * @return the pso best known PID parameters
     */
    public synchronized PIDParams getSwarmOptimum() {
        return swarmOptimum;
    }


    /**
     * Updates the swarm's optimum
     * @param swarmOptimum the new optimum
     */
    private synchronized void setSwarmOptimum(PIDParams swarmOptimum) {
        this.swarmOptimum = swarmOptimum;
    }

    /**
     * Checks if the argument provided is better than the
     * swarm's currently best known PID parameters
     * @param particle the particle to check
     */
    synchronized void updateSwarmOptimum(PIDParams particle) {
        if (this.getSwarmOptimum() == null || particle.getValue() < this.getSwarmOptimum().getValue()){
            PIDParams swarm_optimum = new PIDParams(particle.getKP(), particle.getKD(), particle.getKI());
            swarm_optimum.setValue(particle.getValue());
            setSwarmOptimum(swarm_optimum);
            try {
                Controller.CONSOLE.write("New swarm optimum found: " + swarm_optimum.toString() + " mse: " + swarm_optimum.getValue() + "\n", "info");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * @return this.continue_condition
     */
    public boolean isContinue_condition() {
        return continue_condition;
    }

    /**
     * Updated by the user through the GUI to stop the
     * pso algorithm
     */
    public void setContinue_condition(boolean continue_condition) {
        this.continue_condition = continue_condition;
    }

    @Override
    public void run() {
        if (multi_threaded) {
            while (continue_condition) {
                CountDownLatch countDownLatch = new CountDownLatch(nbOfParticles - 1);
                for (Particle particle : particles) {
                    new Thread(() -> {
                        particle.eval();
                        particle.updatePosition();
                        countDownLatch.countDown();
                    }).start();
                }
                try {
                    countDownLatch.await();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else {
            for (int i = 0; i < nbOfParticles; i++) {
                particles[i].eval();
                particles[i].updatePosition();
                try {
                    Controller.CONSOLE.write("updated position of particle " + i + "\n", "info");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

    }

    /**
     * Specifies if the PSO is to run in multi or
     * single threaded mode
     *
     * @param multi_threaded boolean value
     */
    public void setMulti_threaded(boolean multi_threaded) {
        this.multi_threaded = multi_threaded;
    }
}
