package ParticleSwarmOptimizer;

import PIDController.PIDParams;

import java.util.Random;
import java.util.concurrent.CountDownLatch;

public class ParticleSwarmOptimizer {
    /**
     * the number of particles in the swarm
     */
    private int nbOfParticles;
    /**
     * the upper bound for the parameter space.
     * (ie assume K_p,d,i <= 10)
     */
    private final int upperBound = 100;

    /**
     * The lower bound for the parameter space
     * (ie assume K_p,d,i >= 0)
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

    private Particle[] particles;

    public ParticleSwarmOptimizer(int nbOfParticles) {
        this.nbOfParticles = nbOfParticles;
        this.particles = new Particle[nbOfParticles];
        initParticles();
        Thread[] threads = new Thread[nbOfParticles];
        while (true) {
            CountDownLatch countDownLatch = new CountDownLatch(nbOfParticles - 1);
            new Thread(() -> {
                Particle particle = particles[0];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 0);
                //System.out.println("best pid params of particle " + 0 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[1];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 1);
                //System.out.println("best pid params of particle " + 1 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[2];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 2);
                //System.out.println("best pid params of particle " + 2 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[3];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 3);
                //System.out.println("best pid params of particle " + 3 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[4];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 4);
                //System.out.println("best pid params of particle " + 4 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[5];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 5);
                //System.out.println("best pid params of particle " + 5 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[6];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 6);
                //System.out.println("best pid params of particle " + 6 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[7];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 7);
                //System.out.println("best pid params of particle " + 7 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            new Thread(() -> {
                Particle particle = particles[8];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 8);
                //System.out.println("best pid params of particle " + 8 + particle.getBestKnownParams().getValue());
            }).start();
            new Thread(() -> {
                Particle particle = particles[9];
                particle.eval();
                particle.updatePosition();
                System.out.println("updated position of particle " + 9);
                //System.out.println("best pid params of particle " + 9 + particle.getBestKnownParams().getValue());
                countDownLatch.countDown();
            }).start();
            try {
                countDownLatch.await();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            System.out.println("swarm optimum: " + swarmOptimum.toString() + " value: " + swarmOptimum.getValue());
        }
    }

    private void initParticles() {
        for (int i = 0; i < this.nbOfParticles; i++) {
            particles[i] = new Particle(this);
            System.out.println("initialized particle " + i);
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

    int getUpperBound() {
        return upperBound;
    }

    int getLowerBound() {
        return lowerBound;
    }

    synchronized PIDParams getSwarmOptimum() {
        return swarmOptimum;
    }


    private synchronized void setSwarmOptimum(PIDParams swarmOptimum) {
        this.swarmOptimum = swarmOptimum;
    }

    synchronized void updateSwarmOptimum(PIDParams particle) {
        if (this.getSwarmOptimum() == null || particle.getValue() < this.getSwarmOptimum().getValue()){
            PIDParams swarm_optimum = new PIDParams(particle.getKP(), particle.getKD(), particle.getKI());
            swarm_optimum.setValue(particle.getValue());
            setSwarmOptimum(swarm_optimum);
        }
    }

}
