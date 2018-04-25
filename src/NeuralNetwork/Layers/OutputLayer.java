package NeuralNetwork.Layers;

import CarSimulator.CarSimulator;
import Jama.Matrix;
import NeuralNetwork.Neuron.Neuron;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Represents the networks output layer
 *
 * @author Nicolas Dutly
 */
public class OutputLayer extends Layer {

    private double kp;
    private double kd;
    private double ki;
    private double sp = 10;

    /**
     * Creates an output layer with {@code n} neurons
     *
     * @param n number of neurons in the layer
     */
    public OutputLayer(int n) {
        super(--n /*compensate bias*/);
    }

    /**
     * Generates the output layer's neurons,
     * without adding a bias neuron.
     */
    @Override
    protected void generateNeurons() {
        neurons = new Neuron[nbOfNeurons];
        for (int i = 0; i < nbOfNeurons; i++) {
            neurons[i] = new Neuron(prevLayer.getNbOfNeurons());
        }
    }

    /**
     * Returns the output layer's output,
     * represented in an array where the first
     * element is the output of the first neuron
     * of the output layer, the second the second's
     * and so on.
     *
     * @return an array containing the neurons output, rounded on the second decimal values
     */
    public double[] getOutputVector() {
        double[] tmp;
        tmp = new double[nbOfNeurons];
        for (int i = 0; i < nbOfNeurons; i++) {
            tmp[i] = this.neurons[i].getNeuronOutput();
            assert !Double.isNaN(tmp[i]);
        }
        return tmp;
    }

    /**
     * Calculates the delta of the output layer using the
     * back-propagation algorithm (Using stochastic gradient descent)
     * As the output layer uses a softmax function, this method differs
     * from the one in the {@code HiddenLayer}
     */
    @Override
    public void calculate_delta() {
        double deltaArray[];
        deltaArray = new double[nbOfNeurons];
        for (int k = 0; k < nbOfNeurons; k++) {
            deltaArray[k] = get_errors()[k] * getOutputVector()[k] * (1 - getOutputVector()[k]);
        }
        this.delta = new Matrix(deltaArray, nbOfNeurons);
    }

    private double[] get_errors() {

        CarSimulator simulators[] = new CarSimulator[9];

        int flags[] = new int[6];

        for (int i = 0; i < simulators.length; i++) {
            simulators[i] = new CarSimulator();
        }

        Matrix error_vals_new = new Matrix(new double[3][3]);
        Matrix error_vals_old = new Matrix(new double[3][3]);
        Matrix error_vals_final;

        //Calculate old error using old kp
        new Thread(() -> {
            double error = 0;
            double old_error = 0;
            double integral = 0;
            double derivative = 0;
            double dt = 0.001;
            double val;
            for (int i = 0; i < 3; i++) {
                CarSimulator simulator = simulators[i];
                new Thread(simulator).start();
                long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
                while(stop > System.nanoTime()){
                    error = sp - simulator.getSpeed();
                    integral = integral + (error*dt);
                    derivative = (error-old_error) / dt;
                    val =  kp * error + getOutputVector()[1] * derivative + getOutputVector()[2] * integral;
                    System.out.println(val);
                    if (Double.isNaN(val))break;
                    simulator.setAcceleration(val);
                    old_error = error;
                }
                error_vals_old.set(i,0,error);
                flags[0] = 1;
            }


        }).start();


        //Calculate old error using old kd
        new Thread(() -> {
            double error = 0;
            double old_error = 0;
            double integral = 0;
            double derivative = 0;
            double dt = 0.001;
            double val;
            for (int i = 0; i < 3; i++) {
                CarSimulator simulator = simulators[i];
                new Thread(simulator).start();
                long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
                while(stop > System.nanoTime()){
                    error = sp - simulator.getSpeed();
                    integral = integral + (error*dt);
                    derivative = (error-old_error) / dt;
                    val =  getOutputVector()[0] * error + kd * derivative + getOutputVector()[2] * integral;
                    simulator.setAcceleration(val);
                    old_error = error;
                }
                error_vals_old.set(i,1,error);
                flags[1] = 1;
            }


        }).start();

        //Calculate old error using old ki
        new Thread(() -> {
            double error = 0;
            double old_error = 0;
            double integral = 0;
            double derivative = 0;
            double dt = 0.001;
            double val;
            for (int i = 0; i < 3; i++) {
                CarSimulator simulator = simulators[i];
                new Thread(simulator).start();
                long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
                while(stop > System.nanoTime()){
                    error = sp - simulator.getSpeed();
                    integral = integral + (error*dt);
                    derivative = (error-old_error) / dt;
                    val = getOutputVector()[0] * error +  getOutputVector()[1] * derivative +  ki * integral;
                    simulator.setAcceleration(val);
                    old_error = error;
                }
                error_vals_old.set(i,2,error);
                flags[2] = 1;
            }


        }).start();

        //Calculate new error using new kp
        new Thread(() -> {
            double error = 0;
            double old_error = 0;
            double integral = 0;
            double derivative = 0;
            double dt = 0.001;
            double val;
            for (int i = 0; i < 3; i++) {
                CarSimulator simulator = simulators[i];
                new Thread(simulator).start();
                long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
                while(stop > System.nanoTime()){
                    error = sp - simulator.getSpeed();
                    integral = integral + (error*dt);
                    derivative = (error-old_error) / dt;
                    val =  5 * getOutputVector()[0] * error +  5 * getOutputVector()[1] * derivative +  5 * getOutputVector()[2] * integral;
                    simulator.setAcceleration(val);
                    old_error = error;
                }
                error_vals_new.set(i,0,error);
                flags[3] = 1;
            }


        }).start();


        //Calculate new error using new kd
        new Thread(() -> {
            double error = 0;
            double old_error = 0;
            double integral = 0;
            double derivative = 0;
            double dt = 0.001;
            double val;
            for (int i = 0; i < 3; i++) {
                CarSimulator simulator = simulators[i];
                new Thread(simulator).start();
                long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
                while(stop > System.nanoTime()){
                    error = sp - simulator.getSpeed();
                    integral = integral + (error*dt);
                    derivative = (error-old_error) / dt;
                    val = 5 * getOutputVector()[0] * error + 5 * getOutputVector()[1] * derivative + 5 * getOutputVector()[2] * integral;
                    simulator.setAcceleration(val);
                    old_error = error;
                }
                error_vals_new.set(i,1,error);
                flags[4] = 1;
            }


        }).start();

        //Calculate new error using new ki
        new Thread(() -> {
            double error = 0;
            double old_error = 0;
            double integral = 0;
            double derivative = 0;
            double dt = 0.001;
            double val;
            for (int i = 0; i < 3; i++) {
                CarSimulator simulator = simulators[i];
                new Thread(simulator).start();
                long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
                while(stop > System.nanoTime()){
                    error = sp - simulator.getSpeed();
                    integral = integral + (error*dt);
                    derivative = (error-old_error) / dt;
                    val = 5 * getOutputVector()[0] * error + 5 * getOutputVector()[1] * derivative + 5 * getOutputVector()[2] * integral;
                    simulator.setAcceleration(val);
                    old_error = error;
                }
                error_vals_new.set(i,2,error);
                flags[5] = 1;
            }


        }).start();
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        error_vals_final = error_vals_new.minus(error_vals_old);
        double delta_arr[] = new double[3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                delta_arr[i] += error_vals_final.get(j, i);
            }
            delta_arr[i] = Math.abs(delta_arr[i]);
        }
        return delta_arr;
    }

    public void set_old_params(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

}
