package NeuralNetwork.Layers;

import NeuralNetwork.Neuron.BiasNeuron;
import NeuralNetwork.Neuron.InputNeuron;
import NeuralNetwork.Neuron.Neuron;

/**
 * Describes the network's input layer
 *
 * @author Nicolas Dutly
 */
public class InputLayer extends Layer {

    /**
     * Creates an input layer. Note that as the neuron generation usually
     * takes place in {@link #setPrevLayer(Layer)} (so basically never for
     * the input layer), as such the layers Neurons are generated in this constructor.
     *
     * @param nbOfInputs the network's number of inputs
     */
    public InputLayer(int nbOfInputs) {
        super(nbOfInputs);
        generateNeurons();
    }

    /**
     * Overrides the standard {@code generateNeurons()}
     * function to create {@code InputNeuron}
     */
    @Override
    protected void generateNeurons() {
        neurons = new Neuron[nbOfNeurons];
        for (int i = 0; i < nbOfNeurons - 1; i++) {
            neurons[i] = new InputNeuron();
        }
        neurons[nbOfNeurons - 1] = new BiasNeuron();
    }

    /**
     * Sets the input layer's input vector.
     *
     * @param inputs the layers input vector
     */
    public void setInputs(double[] inputs) {
        for (int i = 0; i < this.neurons.length - 1; i++) {
            this.neurons[i].setInput(inputs[i]);
        }
    }

    /**
     * Does not do anything as the Layer is an input layer
     */
    @Override
    public void calculate_delta() {
    }

}
