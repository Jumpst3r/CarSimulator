import CarSimulator.CarSimulator;
import GeneticAlgorithm.GeneticAlgorithm;
import GeneticAlgorithm.AbsErr;
import Jama.Matrix;

import java.util.concurrent.TimeUnit;

public class Main {
    public static void main(String[] args) {
//        GeneticAlgorithm genAlg = new GeneticAlgorithm(50, 100);
        double error;
        double old_error = 0;
        double integral = 0;
        double derivative;
        double dt = 0.2;
        double val;
        int sp = 50;
        CarSimulator simulator = new CarSimulator();
        new Thread(simulator).start();
        AbsErr absErr = new AbsErr(0);
        long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
        while (stop > System.nanoTime()){
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            error = sp - simulator.getSpeed();
            if (error < 0) System.out.println("error negative!");
            integral = integral + (error * dt);
            absErr.setAbsErr(absErr.getAbsErr() + integral);
            derivative = (error - old_error) / dt;
            val = 8 * error;
            simulator.setAcceleration(val);
            old_error = error;
            System.out.println("error: " + error + " val: " + val);
        }
        System.out.println("absErr: " + absErr.getAbsErr());
    }
}
