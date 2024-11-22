import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from scripts.objects.gripper import GripperMotors
import random
import numpy as np

KP_RANGE = (0, 1024)
KI_RANGE = (0, 512)
KD_RANGE = (0, 64)
POPULATION_SIZE = 150
GENERATIONS = 20
MUTATION_RATE = 0.1
MOTOR_ID = 2

latest_generation = os.path.join(os.path.dirname(__file__), 'latest_generation.txt')
train_results = os.path.join(os.path.dirname(__file__), 'train.txt')
if not os.path.exists(train_results):
  with open(train_results, "w") as file:
      pass

class GeneticAlgorithmPID:
    def __init__(self, kp_range, ki_range, kd_range, pop_size, generations, mutation_rate):
        self.kp_range = kp_range
        self.ki_range = ki_range
        self.kd_range = kd_range
        self.pop_size = pop_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.gripper_motors = GripperMotors()
        self.population = self.initialize_population()

    def initialize_population(self):
        return [
            {
                "Kp": random.uniform(*self.kp_range),
                "Ki": random.uniform(*self.ki_range),
                "Kd": random.uniform(*self.kd_range),
                "fitness": float('inf')
            }
            for _ in range(self.pop_size)
        ]

    def evaluate_fitness(self, individual):
        Kp, Ki, Kd = individual["Kp"], individual["Ki"], individual["Kd"]
        fitness = self.gripper_motors.train_apply_current_compensation_PWM_PID(motor_id=MOTOR_ID, kp = Kp, ki = Ki, kd = Kd)
        return fitness

    def select_parents(self, sorted_population, selection_probs):
        parent1 = np.random.choice(sorted_population, p=selection_probs)
        parent2 = np.random.choice(sorted_population, p=selection_probs)

        while parent2 == parent1:
            parent2 = np.random.choice(sorted_population, p=selection_probs)

        return parent1, parent2

    def crossover(self, parent1, parent2):
        return {
            "Kp": random.choice([parent1["Kp"], parent2["Kp"]]),
            "Ki": random.choice([parent1["Ki"], parent2["Ki"]]),
            "Kd": random.choice([parent1["Kd"], parent2["Kd"]]),
            "fitness": float('inf')
        }

    def mutate(self, individual):
        if random.random() < self.mutation_rate:
            individual["Kp"] = random.uniform(*self.kp_range)
        if random.random() < self.mutation_rate:
            individual["Ki"] = random.uniform(*self.ki_range)
        if random.random() < self.mutation_rate:
            individual["Kd"] = random.uniform(*self.kd_range)
        return individual
      
    def get_sorted_population_and_probabilities(self):
      sorted_population = sorted(self.population, key=lambda ind: ind["fitness"])
      
      fitness_values = np.array([ind["fitness"] for ind in sorted_population])
      selection_probs = 1 / (fitness_values + 1e-6)
      selection_probs /= selection_probs.sum()
      
      return sorted_population, selection_probs
    
    def evaluate_fitnesses(self):
      for i, individual in enumerate(self.population):
            individual["fitness"] = self.evaluate_fitness(individual)
            print(f"Individual {i} fitness is {individual['fitness']}")
  
    def evolve_population(self):
        new_population = []
        sorted_population, selection_probs = self.get_sorted_population_and_probabilities()

        for _ in range(self.pop_size):
            parent1, parent2 = self.select_parents(sorted_population, selection_probs)
            child = self.crossover(parent1, parent2)
            child = self.mutate(child)
            new_population.append(child)
        
        self.population = new_population

ga_pid = GeneticAlgorithmPID(KP_RANGE, KI_RANGE, KD_RANGE, POPULATION_SIZE, GENERATIONS, MUTATION_RATE)
if os.path.exists(latest_generation):
  with open(latest_generation, "r") as file:
        ga_pid.population = [
            {
                "Kp": float(line.split(",")[0].strip()),
                "Ki": float(line.split(",")[1].strip()),
                "Kd": float(line.split(",")[2].strip()),
                "fitness": float(line.split(",")[3].strip())
            }
            for line in file
        ]

for generation in range(1, GENERATIONS + 1):
    ga_pid.evaluate_fitnesses()

    best_individual = min(ga_pid.population, key=lambda ind: ind["fitness"])
    print(f"Generation {generation}/{GENERATIONS}")
    print(f"Best Kp: {best_individual['Kp']}, Ki: {best_individual['Ki']}, Kd: {best_individual['Kd']}")
    print(f"Fitness: {best_individual['fitness']}")
    with open(train_results, "a+") as file:
      file.write(f"{best_individual['fitness']}\n")
    
    # Save latest generation for continued training in the future
    with open(latest_generation, "w") as file:
      for individual in ga_pid.population:
        file.write(f"{individual['Kp']}, {individual['Ki']}, {individual['Kd']}, {individual['fitness']}\n")
        
    ga_pid.evolve_population()

ga_pid.gripper_motors.disable_torque(MOTOR_ID)