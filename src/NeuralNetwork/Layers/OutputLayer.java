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
            tmp[i] = Math.round(tmp[i] * 100);
            tmp[i] = tmp[i] / 100;
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
            //note the different derivative (a soft-max function is used in the output layer)
            deltaArray[k] = (this.trainingOutput[k] - neurons[k].getNeuronOutput());
        }
        this.delta = new Matrix(deltaArray, nbOfNeurons);

    }

    private double[] get_errors() {
        double error;
        double old_error = 0;
        double integral = 0;
        double derivative = 0;
        double dt = 1;
        CarSimulator simulator = new CarSimulator();
        new Thread(simulator).start();
        double val;
        //calculate using old values
        for (int i = 0; i <= 3; i++) {
            long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(2+i);
            while(stop > System.nanoTime()){
                error = sp - simulator.getSpeed();
                integral += error;
                derivative = (error-old_error) / dt;
                val = kp * error + kd * derivative + ki * integral;
                simulator.setAcceleration(val);
                old_error = error;
                System.out.printf("error: %f\n", error);
            }
        }


    }

    public void set_old_params(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

}
