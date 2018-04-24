package NeuralNetwork.Layers;

import Jama.Matrix;

/**
 * Describes the network's hidden layer.
 *
 * @author Nicolas Dutly
 */
public class HiddenLayer extends Layer {
    /**
     * Creates a hidden Layer
     *
     * @param nbOfNeurons amount of neurons in the layer
     */
    public HiddenLayer(int nbOfNeurons) {
        super(nbOfNeurons);
    }

    /**
     * Calculates the delta for the hidden layer using SGD and as such
     * the sigmoid derivatives
     */
    @Override
    public void calculate_delta() {
        double[] tmp;
        double weighted_delta_sum;
        int nbOfNextNeurons;
        weighted_delta_sum = 0;
        tmp = new double[nbOfNeurons];
        //if next layer is a HL, don't iterate over the last Neuron (Layer's bias)
        nbOfNextNeurons = (nextLayer.getClass().getCanonicalName().equals("NeuralNetwork.Layers.OutputLayer")) ? nextLayer.getNbOfNeurons() : nextLayer.getNbOfNeurons() - 1;
        for (int j = 0; j < nbOfNeurons; j++) {
            for (int k = 0; k < nbOfNextNeurons; k++) {
                weighted_delta_sum += nextLayer.getNeurons()[k].getSynapse_weights().get(j, 0) * nextLayer.getDelta().get(k, 0);
            }
            tmp[j] = this.neurons[j].getNeuronOutput() * (1 - this.getNeurons()[j].getNeuronOutput()) * weighted_delta_sum;
        }
        this.delta = new Matrix(tmp, nbOfNeurons);
    }
}