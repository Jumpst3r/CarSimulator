import CarSimulator.CarSimulator;
import Jama.Matrix;
import ParticleSwarmOptimizer.ParticleSwarmOptimizer;

import java.io.*;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Main {
    public static void main(String[] args) {
        ParticleSwarmOptimizer pso = new ParticleSwarmOptimizer(10);
        /*double error;
        double old_error = 0;
        double integral = 0;
        double derivative;
        double dt = 0.02;
        double val;
        int sp = 50;
        CarSimulator simulator = new CarSimulator();
        new Thread(simulator).start();
        long stop = System.nanoTime() + TimeUnit.SECONDS.toNanos(10);
        int n = 0;
        double mse = 0;
        ArrayList<Double> speedTable = new ArrayList<>();
        while (stop > System.nanoTime()) {
            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            speedTable.add(simulator.getSpeed());
            error = sp - simulator.getSpeed();
            //System.out.println("error: " + error);
            mse += Math.pow(error, 2);
            n++;
            //System.out.println("mse: " + mse);
            integral = integral + (error * dt);
            //System.out.println("integral" + integral);
            derivative = (error - old_error) / dt;
            //System.out.println("derivative " + derivative);
            val = 8.607930653754462 * error + 0.22714051907477772 * derivative + 4.163634243740372 * integral;
            //System.out.println("val: " + val);
            simulator.setAcceleration(val > 10 ? 10 : val);
            old_error = error;
            System.out.println("error: " + error);
        }
        simulator.stop();
        File file = new File("error.csv");
        BufferedWriter bf = null;
        try {
            bf = new BufferedWriter(new FileWriter(file));
            for (int i = 0; i < speedTable.size(); i++) {
                bf.write(i + ", " + speedTable.get(i) + "\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }finally {
            try {
                bf.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }*/
    }
}
