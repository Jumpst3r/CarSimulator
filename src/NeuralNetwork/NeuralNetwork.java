package NeuralNetwork;

import Jama.Matrix;
import NeuralNetwork.Layers.HiddenLayer;
import NeuralNetwork.Layers.InputLayer;
import NeuralNetwork.Layers.OutputLayer;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Random;
import java.util.regex.Pattern;


/**
 * Represents a neural network containing one input layer with 3 inputs,
 * two hidden layer with a variable number of neurons and one output layer
 * containing 3 neurons
 * <p>
 * <p>This project was created for the ROB[SP17] course and adapted for the PC[SP18] to find
 * optimal hyperparameters for the PID controller</p>
 * <p>
 * <p><b>Note:</b>The  external libraries used are <a href="http://math.nist.gov/javanumerics/jama/">JAMA</a>,
 * which provides basic linear algebra functions. In this project, JAMA is mostly used to store information
 * in matrices, which makes the calculations more readable.</p>
 * <p>
 * Network type: FFANN  (Feed forward artificial neural network)<br>
 * BP Algorithm: SGD (Stochastic gradient descent)<br>
 * Learning type: unsupervised<br>
 * </p>
 * <p>
 *
 * @author Nicolas Dutly
 * @version 1.0
 */


public class NeuralNetwork {
    /**
     * Represents the networks bias value
     * (Every neuron has an additional bias input
     * with the value of {@code BIAS})
     */
    public static final int BIAS = -1;

    /**
     * Represents the networks input layer
     */
    private InputLayer inputLayer;
    /**
     * Represents the networks first hidden layer
     */
    private HiddenLayer hiddenLayer1;
    /**
     * Represents the networks second hidden layer
     */
    private HiddenLayer hiddenLayer2;
    /**
     * Represents the networks output layer
     */
    private OutputLayer outputLayer;

    /**
     * Create a neural network with the following topology:
     * Input Layer: 3 input nodes (R/G/B) + bias
     * 2 hidden Layer with a variable amount of neurons (-n flag)
     * Output Layer: variable amount of neurons (-c flag)
     *
     * @param nbOfHiddenNeurons specifies the number of hidden layer neurons
     */
    public NeuralNetwork(int nbOfHiddenNeurons) {
        //note the input layer bias is created in the InputLayer class
        this.inputLayer = new InputLayer(3);
        this.hiddenLayer1 = new HiddenLayer(nbOfHiddenNeurons);
        this.hiddenLayer2 = new HiddenLayer(nbOfHiddenNeurons);

        this.hiddenLayer1.setPrevLayer(inputLayer);
        this.hiddenLayer1.setNextLayer(this.hiddenLayer2);
        this.hiddenLayer2.setPrevLayer(hiddenLayer1);

        this.outputLayer = new OutputLayer(3);
        this.outputLayer.setPrevLayer(hiddenLayer2);

        this.hiddenLayer2.setNextLayer(this.outputLayer);

    }

    public void init(int nbOfEpochs) {
        inputLayer.setInputs(new double[]{0.5, 0.2, 0.8, BIAS});
        outputLayer.set_old_params(0.2,0.5,0.4);
        for (int i = 0; i < nbOfEpochs; i++) {
            printProgress(nbOfEpochs, i);
            train();
        }

    }

    /**
     * Print the current training progress percentage
     *
     * @param nbOfEpochs   total number of epochs
     * @param currentEpoch elapsed epochs
     */
    private void printProgress(int nbOfEpochs, int currentEpoch) {
        double percentage = (100. / (double) nbOfEpochs) * currentEpoch;
        System.out.printf("\r[%.2f%%] KP: %f, KD: %f, KI: %f", percentage, outputLayer.getOutputVector()[0], outputLayer.getOutputVector()[1], outputLayer.getOutputVector()[2]);
    }

    /**
     * Feeds a the training set through the neural network,
     * then back-propagates the error and adjusts the weights.
     * Called once during every epoch.
     *
     * @see #init(int)
     */
    private void train() {

        //forward phase
        hiddenLayer1.process();
        hiddenLayer2.process();
        outputLayer.process();


        //Back propagation
        outputLayer.calculate_delta();
        hiddenLayer2.calculate_delta();
        hiddenLayer1.calculate_delta();

        outputLayer.adjustLayerWeights();
        hiddenLayer2.adjustLayerWeights();
        hiddenLayer1.adjustLayerWeights();

        inputLayer.setInputs(new double[]{outputLayer.getOutputVector()[0], outputLayer.getOutputVector()[1], outputLayer.getOutputVector()[2], BIAS});
        outputLayer.set_old_params(outputLayer.getOutputVector()[0], outputLayer.getOutputVector()[1], outputLayer.getOutputVector()[2]);

    }
}
