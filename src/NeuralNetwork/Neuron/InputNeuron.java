package NeuralNetwork.Neuron;

import Jama.Matrix;

/**
 * Represents the networks input "Neurons", as such
 * does not contain synapse weights and does not
 * provide most of a standard neurons functionality.
 * @author Nicolas Dutly
 */
public class InputNeuron extends Neuron {
    /**
     * Create a new input neuron
     */
    public InputNeuron() {
        super(1);
    }

    @Override
    void calculateWeightedInput() {
        System.err.println("Invalid op on input neuron");
        System.exit(0);
    }

    @Override
    double activate(double weightedInput) {
        return this.input.get(0, 0);
    }

    @Override
    public void process(Matrix input) {
        System.err.println("Invalid op on input neuron");
        System.exit(0);
    }

    @Override
    public void adjustSynapseWeights(Matrix weightDelta) {
        System.err.println("Invalid op on input neuron");
        System.exit(0);
    }

    @Override
    public double getNeuronOutput() {
        return this.input.get(0, 0);
    }

    @Override
    public Matrix getSynapse_weights() {
        System.err.println("Invalid op on input neuron");
        System.exit(0);
        return null;
    }
    @Override
    public void setInput(double input) {
        this.input = new Matrix(1, 1);
        this.input.set(0, 0, input);
    }

    /**
     * Override the neuron's output. Should only be used
     * in the output layer in tandem with the soft-max function
     *
     * @param neuronOutput new output value
     */
    @Override
    public void setNeuronOutput(double neuronOutput) {
        System.err.println("Invalid op on input neuron");
        System.exit(0);
    }

    @Override
    public double getWeightedInput() {
        System.err.println("Invalid op on input neuron");
        System.exit(0);
        return 0;
    }
}
