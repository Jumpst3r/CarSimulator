import CarSimulator.CarSimulator;
import NeuralNetwork.NeuralNetwork;

public class Main {
    public static void main(String[] args) {
        NeuralNetwork neuralNetwork = new NeuralNetwork(5);
        neuralNetwork.init(10);
    }
}
