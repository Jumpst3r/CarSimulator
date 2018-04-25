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
        /*for (int i = 0; i < genes.length; i++) {
            Random r = new Random();
            genes[i] = 5.0 * r.nextDouble();
        }*/
        genes[0] = 10.0 * r.nextDouble();
        genes[1] = 0;
        genes[2] = 0;
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
            double dt = 0.001;
            double val;
            int sp = 5;
            CarSimulator simulator = new CarSimulator();
            new Thread(simulator).start();
            long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
            while (stop > System.nanoTime()){
                error = sp - simulator.getSpeed();
                integral = integral + (error * dt);
                absErr.setAbsErr(absErr.getAbsErr() + (error * dt));
                derivative = (error - old_error) / dt;
                val = dna.genes[0] * error + dna.genes[1] * derivative + dna.genes[2] * integral;
                simulator.setAcceleration(val);
                old_error = error;
            }
            simulator.stop();
        }).start();
        try {
            Thread.sleep(6000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        absErr.setAbsErr(Math.abs(absErr.getAbsErr()));
        System.out.println("absError: " + absErr.getAbsErr());
        if (absErr.getAbsErr() == 0.5) System.out.println("defaulted!");
//        absErr.setAbsErr(absErr.getAbsErr() / 10000);
        if (absErr.getAbsErr() < 1) return 1;
        double delta =  Math.abs(1 / absErr.getAbsErr());
        if (delta > 1) return 1;
        return delta * 10;
    }

    /**
     * mix this DNA with genA.
     * Calculates the mean of the PID
     * params
     * @param genA
     * @return the child DNA
     */
    public DNA crossOver(DNA genA) {
        double n_KP = (genA.genes[0] + this.genes[0]) / 2;
        double n_KD = 0;//(genA.genes[1] + this.genes[1]) / 2;
        double n_KI = 0;//(genA.genes[2] + this.genes[2]) / 2;

        DNA child = new DNA(new PIDParams(n_KP, n_KD, n_KI));
        return mutate(child);
    }

    /**
     * slightly mutates a given DNA
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
