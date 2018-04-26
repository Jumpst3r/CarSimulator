package GeneticAlgorithm;

import CarSimulator.CarSimulator;
import Jama.Matrix;
import PIDController.PIDParams;

import java.util.Random;
import java.util.concurrent.TimeUnit;

public class DNA {
    private double[] genes = new double[3];
    private double fitness;
    private PIDParams phenotype;

    public DNA(PIDParams params) {
        genes[0] = params.getKP();
        genes[1] = params.getKD();
        genes[2] = params.getKI();
        this.phenotype = getPhenotype();
        this.fitness = calculateFitness(this);
        System.out.printf("DNA PIDParams: %f, %f, %f with fitness %f\n", genes[0], genes[1], genes[2], this.fitness);
    }

    public DNA() {
        Random r = new Random();
        genes[0] = 10.0 * r.nextDouble();
        genes[1] = 5.0 * r.nextDouble();
        genes[2] = 5.0 * r.nextDouble();
        this.phenotype = getPhenotype();
        this.fitness = calculateFitness(this);
        System.out.printf("DNA PIDParams: %f, %f, %f with fitness %f\n", genes[0], genes[1], genes[2], this.fitness);
    }

    public PIDParams getPhenotype() {
        return new PIDParams(genes[0], genes[1], genes[2]);
    }

    private double calculateFitness(DNA dna) {
        //Calculate old error using old kp
        AbsErr absErr = new AbsErr(0.5);
        new Thread(() -> {
            double error;
            double old_error = 0;
            double integral = 0;
            double derivative;
            double dt = 0.1;
            double val;
            int sp = 50;
            CarSimulator simulator = new CarSimulator();
            new Thread(simulator).start();
            long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            int n = 0;
            double mse = 0;
            while (stop > System.nanoTime()) {
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                error = sp - simulator.getSpeed();
                //System.out.println("error: " + error);
                mse += Math.pow(error, 2);
                n++;
                //System.out.println("mse: " + mse);
                integral = integral + (error * dt);
                //System.out.println("integral" + integral);
                derivative = (error - old_error) / dt;
                //System.out.println("derivative " + derivative);
                val = dna.genes[0] * error + dna.genes[1] * derivative + dna.genes[2] * integral;
                //System.out.println("val: " + val);
                simulator.setAcceleration(val);
                old_error = error;
            }
            absErr.setAbsErr(mse);
            simulator.stop();
        }).start();
        try {
            Thread.sleep(2500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (absErr.getAbsErr() < 1) return 1;
        double max = 1E3;
        if (absErr.getAbsErr() > 1E3) max = absErr.getAbsErr();
        return 1 - Math.abs(absErr.getAbsErr() / 50000) < 0 ? 0 : 1 - Math.abs(absErr.getAbsErr() / 50000);
    }

    /**
     * mix this DNA with genA.
     * Calculates the mean of the PID
     * params
     *
     * @param genA
     * @return the child DNA
     */
    public DNA[] crossOver(DNA genA) {
        double alpha = 0.8;
        double n_KP = alpha * genA.genes[0] + (1 - alpha) * this.genes[0];
        double n_KD = alpha * genA.genes[1] + (1 - alpha) * this.genes[1];
        double n_KI = alpha * genA.genes[2] + (1 - alpha) * this.genes[2];

        DNA child1 = new DNA(new PIDParams(n_KP, n_KD, n_KI));

        double n_KP2 = alpha * this.genes[0] + (1 - alpha) * genA.genes[0];
        double n_KD2 = alpha * this.genes[1] + (1 - alpha) * genA.genes[1];
        double n_KI2 = alpha * this.genes[2] + (1 - alpha) * genA.genes[2];

        DNA child2 = new DNA(new PIDParams(n_KP2, n_KD2, n_KI2));
        mutate(child1);
        mutate(child2);
        return new DNA[]{child1, child2};
    }

    /**
     * slightly mutates a given DNA
     *
     * @param child
     * @return
     */
    private DNA mutate(DNA child) {
        Random rnd = new Random();
        if (rnd.nextDouble() < GeneticAlgorithm.MUTATION_RATE) {
            int i = rnd.nextInt(3);
            child.genes[0] = 5.0 * rnd.nextDouble();
            child.phenotype = new PIDParams(child.genes[0], child.genes[1], child.genes[2]);
            child.fitness = calculateFitness(child);
            System.out.println("mutated!");
        }
        return child;
    }

    public double getFitness() {
        return fitness;
    }
}
