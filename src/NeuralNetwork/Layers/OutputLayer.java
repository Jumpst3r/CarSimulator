package NeuralNetwork.Layers;

import Jama.Matrix;
import NeuralNetwork.Neuron.Neuron;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Represents the networks output layer
 *
 * @author Nicolas Dutly
 */
public class OutputLayer extends Layer {
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


    /**
     * Used to retrieve the training or validation classification error
     * of the current epoch.
     *
     * @param trainingErr if set to true returns the training classification error, otherwise the validation
     *                    classification error
     * @return the classification error percentage (Ex: 5% means that 5% of the data lines i
     * the training or validation (depending on the param) where incorrectly classified)
     */
    public double get_class_err(boolean trainingErr) {
        double error_rate;
        if (trainingErr) {
            error_rate = (1 - (tr_correct / proccessed_training_sets)) * 100;
            proccessed_training_sets = 0;
            tr_correct = 0;
        } else {
            error_rate = (1 - (valid_correct / proccessed_validation_sets)) * 100;
            proccessed_validation_sets = 0;
            valid_correct = 0;
        }
        return error_rate;
    }

    /**
     * Specifies the layers training output
     *
     * @param trainingOutput array containing the training output
     */
    public void setTrainingOutput(double[] trainingOutput) {
        this.trainingOutput = trainingOutput;
    }


}
