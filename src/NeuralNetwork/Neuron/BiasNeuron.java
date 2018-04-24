package NeuralNetwork.Neuron;

import Jama.Matrix;
import NeuralNetwork.NeuralNetwork;

/**
 * Stripped down version of the
 * NeuralNetwork.Neuron class, represents a
 * neuron with static input
 * @author Nicolas Dutly
 */
public class BiasNeuron extends Neuron {

    public BiasNeuron() {
        super(1);
    }

    /**
     * The bias neuron has a static input of one.
     * This method ignores all other passed input
     * and sets the bias neurons input statically.
     *
     * @param input the neurons input Matrix (no effect)
     * @see #activate(double)
     */
    @Override
    public void process(Matrix input) {

    }

    @Override
    public void adjustSynapseWeights(Matrix weightDelta) {

    }

    @Override
    void calculateWeightedInput() {

    }

    @Override
    double activate(double weightedInput) {
        return NeuralNetwork.BIAS;
    }

    @Override
    public Matrix getSynapse_weights() {
       return null;
    }

    @Override
    public double getWeightedInput() {
        return 0;
    }

    @Override
    public void setNeuronOutput(double neuronOutput) {}

    @Override
    public double getNeuronOutput() {
        return NeuralNetwork.BIAS;
    }
}