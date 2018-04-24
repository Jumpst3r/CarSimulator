package NeuralNetwork.Neuron;

import Jama.Matrix;

import java.util.concurrent.ThreadLocalRandom;

/**
 * Represent the implementation of a neuron.
 * Neurons use a sigmoid activation function
 * by default (Except in the output layer)
 *
 * @author Nicolas Dutly
 * @see #activate(double)
 */
public class Neuron {
    /**
     * Describes a matrix containing the neurons input
     */
    protected Matrix input;
    /**
     * Represents the weights of the inputs of the neuron
     */
    private Matrix synapse_weights;
    /**
     * Contains the neurons output [0-1]
     */
    private double neuronOutput;
    /**
     * Contains the sum of the inputs multiplied with their respective weights
     */
    private double weightedInput;
    /**
     * Describes the amount of inputs for a given neuron
     */
    private int nb_of_inputs;

    /**
     * Creates a new neuron
     *
     * @param nb_of_inputs number of neuron inputs
     */
    public Neuron(int nb_of_inputs) {
        this.nb_of_inputs = nb_of_inputs;
        try {
            this.synapse_weights = new Matrix(setRandomWeights(), this.nb_of_inputs);
        } catch (Exception e) {
            System.out.println("Tried to assign random weights to non null weights!");
            e.printStackTrace();
        }
    }

    /**
     * Generates null centered random double between [-1/sqrt(n),1/sqrt(n)],
     * where n the number of neuron inputs
     * Only called upon neuron creation
     *
     * @return the generated weight matrix
     * @throws Exception when trying to assign random weights to weight that have been previously
     *                   initialized.
     */
    private double[] setRandomWeights() throws Exception {
        if (this.synapse_weights != null)
            throw new Exception("Attempting to assign random weights to non-null weights!");
        double[] weights = new double[nb_of_inputs];
        for (int i = 0; i < weights.length; i++) {
            weights[i] = ThreadLocalRandom.current().nextDouble((-1. / Math.sqrt(nb_of_inputs)), (1. / Math.sqrt(nb_of_inputs) + 0.01));
        }
        return weights;
    }

    /**
     * Calculates the neurons weighted input by summing up the inputs multiplied
     * with their given weights.
     */
    void calculateWeightedInput() {
        Matrix tmp = this.input.arrayTimes(this.synapse_weights);
        double sum = 0;
        for (int i = 0; i < this.input.getRowDimension(); i++) {
            sum += tmp.get(i, 0);
        }
        this.weightedInput = sum;
    }

    /**
     * Passes the weighted input to the sigmoid function,
     * setting {@link #neuronOutput} to the output of the
     * sigmoid function.
     *
     * @param weightedInput The neurons weighted input
     * @return The neurons output
     */
    double activate(double weightedInput) {
        return (1. / (1 + Math.exp(-weightedInput)));
    }

    /**
     * Processes an input matrix, calculating the weighted input and
     * passing it to the through the activation function
     *
     * @param input the neurons input Matrix (nb_of_inputs x 1)
     * @see #activate(double)
     */
    public void process(Matrix input) {
        this.input = input;
        calculateWeightedInput();
        this.neuronOutput = activate(this.weightedInput);
    }

    /**
     * Adjusts neurons weight matrix by a given delta.
     * Used in the back-propagation algorithm
     *
     * @param weightDelta The delta Matrix to be added to the
     *                    neurons weights.
     */
    public void adjustSynapseWeights(Matrix weightDelta) {
        double learningRate = 1E-3;
        this.synapse_weights.plusEquals(weightDelta.times(learningRate));
    }

    /**
     * @return the neurons output
     */
    public double getNeuronOutput() {
        return neuronOutput;
    }

    /**
     * Override the neuron's output. Should only be used
     * in the output layer in tandem with the soft-max function
     *
     * @param neuronOutput new output value
     */
    public void setNeuronOutput(double neuronOutput) {
        this.neuronOutput = neuronOutput;
    }

    /**
     * @return the neurons weight matrix
     */
    public Matrix getSynapse_weights() {
        return synapse_weights;
    }

    /**
     * @return the neuron's weighted input
     */
    public double getWeightedInput() {
        return weightedInput;
    }

    /**
     * Manually sets the neuron's input, overridden only be {@code InputNeuron}
     * @param in input to be set
     */
    public void setInput(double in) {

    }
}

