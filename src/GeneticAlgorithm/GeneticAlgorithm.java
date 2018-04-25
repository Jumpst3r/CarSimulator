package GeneticAlgorithm;

import java.util.ArrayList;
import java.util.Random;

public class GeneticAlgorithm {
    static final double MUTATION_RATE = 0.01;
    private int popSize;
    private DNA[] population;
    private ArrayList<DNA> matingPool = new ArrayList<>();
    private int generations;

    public GeneticAlgorithm(int popSize, int generations) {
        this.popSize = popSize;
        this.generations = generations;
        population = new DNA[popSize];
        for (int i = 0; i < popSize; i++) {
            population[i] = new DNA();
        }
        evolve();
    }

    private void evolve() {
        for (int i = 0; i < generations; i++) {
            System.out.printf("--Gen%d--\n", i);
            fillMatingPool();
            mixMatingPool();
            checkPopulation();
        }
    }

    private void checkPopulation() {
        double max = 0;
        int index = 0;
        for (int i = 0; i < popSize; i++) {
            if (population[i].getFitness() > max){
                max = population[i].getFitness();
                index = i;
            }
        }
        System.out.println("Generation statistics:");
        System.out.println(population[index].getPhenotype().toString());
        System.out.printf("max fitness: %f\n", max);
    }

    private void fillMatingPool() {
        matingPool.clear();
        for (DNA genome : population) {
            int n = (int) (genome.getFitness() * 100);
            for (int i = 0; i < n; i++) {
                matingPool.add(genome);
            }
        }
    }

    private void mixMatingPool() {
        Random rnd = new Random();
        int n;
        int k;
        for (int i = 0; i < population.length; i++) {
            n = rnd.nextInt(matingPool.size());
            k = rnd.nextInt(matingPool.size());
            population[i] = matingPool.get(n).crossOver(matingPool.get(k));
        }
    }



}
