package NeuralNetwork.Layers;

import Jama.Matrix;
import NeuralNetwork.Neuron.BiasNeuron;
import NeuralNetwork.Neuron.Neuron;

/**
 * Describes Layer in the Neural network
 *
 * @author Nicolas Dutly
 */
abstract public class Layer {

    /**
     * Describes an array of neurons, containing
     * the neurons in the output layer
     */
    protected Neuron[] neurons;
    /**
     * Describes the number of neurons
     * in the Layer.
     */
    protected int nbOfNeurons;
    /**
     * Describes the Matrix containing the delta
     * for each neuron (used in back-propagation)
     */
    protected Matrix delta;
    /**
     * Describes the previous layer
     * of type HiddenLayer
     */
    Layer prevLayer;
    /**
     * Describes the next Layer after this one (if any)
     */
    Layer nextLayer;

    /**
     * @param nbOfNeurons number of neurons in the layer
     */
    public Layer(int nbOfNeurons) {
        //add the bias. Up to the OutputLayer to compensate (OL has no bias)
        this.nbOfNeurons = ++nbOfNeurons;
    }

    /**
     * Generates the neurons in the layer, with a bias neuron.
     * This method is overridden by the output layer,
     * as it contains no bias
     * @see BiasNeuron
     */
    protected void generateNeurons() {
        neurons = new Neuron[nbOfNeurons];
        for (int i = 0; i < nbOfNeurons - 1; i++) {
            neurons[i] = new Neuron(prevLayer.getNbOfNeurons());
        }
        neurons[nbOfNeurons - 1] = new BiasNeuron();
    }

    /**
     * Calculates the delta using the
     * back-propagation algorithm (Using stochastic gradient descent).
     * The calculation is different on hidden layers and output layers,
     * as such this method has to be implemented by the corresponding layer.
     */
    abstract public void calculate_delta();

    /**
     * Adjusts the weights of the individual synapses using the delta calculated in
     *
     * @see #calculate_delta()
     */
    public void adjustLayerWeights() {
        double[] tmpw;
        tmpw = new double[prevLayer.getNbOfNeurons()];
        for (int k = 0; k < nbOfNeurons; k++) {
            for (int j = 0; j < prevLayer.getNbOfNeurons(); j++) {
                tmpw[j] = delta.get(k, 0) * prevLayer.getNeurons()[j].getNeuronOutput();
            }
            Matrix weightDelta = new Matrix(tmpw, prevLayer.getNbOfNeurons());
            neurons[k].adjustSynapseWeights(weightDelta);
        }
    }


    /**
     * Activates each neuron in Layer (except the bias)
     */
    public void process() {
        double[] tmp2;
        for (int k = 0; k < this.nbOfNeurons; k++) {
            tmp2 = new double[prevLayer.getNbOfNeurons()];
            for (int j = 0; j < prevLayer.getNbOfNeurons(); j++) {
                tmp2[j] = prevLayer.getNeurons()[j].getNeuronOutput();
            }
            this.neurons[k].process(new Matrix(tmp2, prevLayer.getNbOfNeurons()));
        }
    }

    /**
     * @return layer's number of neurons
     */
    public int getNbOfNeurons() {
        return nbOfNeurons;
    }

    /**
     * @return the layer's delta Matrix
     */
    public Matrix getDelta() {
        return delta;
    }

    /**
     * @return the layers neuron array
     */
    public Neuron[] getNeurons() {
        return neurons;
    }

    /**
     * When the previous layer is specified, the layer has enough information
     * to generate its neurons, which take place in the function, by calling
     * {@link #generateNeurons()}
     * {@code setPrevLayer(Layer prevLayer)} is not called
     * in the input layer, as such the input layer's neuron generation
     * takes place in it
     * @param prevLayer The previous Layer
     */
    public void setPrevLayer(Layer prevLayer) {
        this.prevLayer = prevLayer;
        generateNeurons();
    }

    /**
     * @param nextLayer The next Layer
     */
    public void setNextLayer(Layer nextLayer) {
        this.nextLayer = nextLayer;
    }

}
