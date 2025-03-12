#!/usr/bin/env python3
from mchgenalg import GeneticAlgorithm
import mchgenalg
import numpy as np
import os
from pathlib import Path
import subprocess

timesEvaluated = 0
bestrmse = -1

def fitness_function(genome):
    global timesEvaluated, bestrmse
    timesEvaluated += 1
    print(f"Fitness function invoked {timesEvaluated} times")

    # Setting parameter values using genome
    # outlier_rejection_quantile in keyframe_ba_monolid.launch
    out_rej_quant = decode_function(genome[0:10])
    if out_rej_quant > 1:
        out_rej_quant = 1

    # max_number_landmarks_near_bin in keyframe_ba_monolid.launch
    max_number_landmarks_near_bin = decode_function(genome[11:22]) * 1000
    if max_number_landmarks_near_bin > 999:
        max_number_landmarks_near_bin = 999.0

    # max_number_landmarks_middle_bin in keyframe_ba_monolid.launch
    max_number_landmarks_middle_bin = decode_function(genome[23:33]) * 1000
    if max_number_landmarks_middle_bin > 999:
        max_number_landmarks_middle_bin = 999.0

    # max_number_landmarks_far_bin in keyframe_ba_monolid.launch
    max_number_landmarks_far_bin = decode_function(genome[34:44]) * 1000
    if max_number_landmarks_far_bin > 999:
        max_number_landmarks_far_bin = 999.0

    # shrubbery_weight in keyframe_ba_monolid.launch
    shrubbery_weight = decode_function(genome[45:55])
    if shrubbery_weight > 1:
        shrubbery_weight = 1

    print("Saving parameters to params.yaml...")
    params_path = Path("/tmp/params.yaml")
    with params_path.open("w", encoding="utf-8") as output:
        output.write(f"outlier_rejection_quantile: {out_rej_quant}\n")
        output.write(f"max_number_landmarks_near_bin: {max_number_landmarks_near_bin}\n")
        output.write(f"max_number_landmarks_middle_bin: {max_number_landmarks_middle_bin}\n")
        output.write(f"max_number_landmarks_far_bin: {max_number_landmarks_far_bin}\n")
        output.write(f"shrubbery_weight: {shrubbery_weight}\n")

    # Call external script to calculate rmse value
    query = "./script.sh"
    print(f"Running command: {query}")
    subprocess.run(query, shell=True, check=True)

    # Read fitness value as root mean square value (rmse) from text files
    rmse1_path = Path("/tmp/rmse_output1.txt")
    rmse2_path = Path("/tmp/rmse_output2.txt")
    with rmse1_path.open("r", encoding="utf-8") as file:
        rmse1 = float(file.read().strip())
    with rmse2_path.open("r", encoding="utf-8") as file:
        rmse2 = float(file.read().strip())

    # Average of the two RMSE values
    rmse_avg = (rmse1 + rmse2) / 2

    print("Saving fitnesses for each evaluation")
    fitnesses_path = Path("/tmp/fitnesses_dump.txt")
    with fitnesses_path.open("a", encoding="utf-8") as output:
        output.write(f"{timesEvaluated} {rmse1} {rmse2} {rmse_avg}\n")

    # Remove rmse output files if they exist
    for rmse_file in (rmse1_path, rmse2_path):
        if rmse_file.exists():
            rmse_file.unlink()

    # Update best fitness if the current run is better
    if bestrmse == -1 or rmse_avg < bestrmse:
        bestrmse = rmse_avg
        best_params_path = Path("/tmp/BestParameters.txt")
        with best_params_path.open("a", encoding="utf-8") as output:
            output.write(f"Best rmse value : {bestrmse}\n")
            output.write(f"outlier_rejection_quantile = {out_rej_quant}\n")
            output.write(f"max_number_landmarks_near_bin = {max_number_landmarks_near_bin}\n")
            output.write(f"max_number_landmarks_middle_bin = {max_number_landmarks_middle_bin}\n")
            output.write(f"max_number_landmarks_far_bin = {max_number_landmarks_far_bin}\n")
            output.write(f"shrubbery_weight = {shrubbery_weight}\n")
            output.write("=================================================\n")

    print(f"Average rmse for this run: {rmse_avg}")
    print(f"Best rmse so far: {bestrmse}")
    return 1 / rmse_avg

def decode_function(genome_partial):
    prod = 0
    n = len(genome_partial)
    # Iterate in reverse order
    for i, e in reversed(list(enumerate(genome_partial))):
        if e:
            prod += 2 ** abs(i - n + 1)
    return prod / 1000

def main():
    # Remove previous fitness dump file if it exists
    fitnesses_dump_path = Path("/tmp/fitnesses_dump.txt")
    if fitnesses_dump_path.exists():
        fitnesses_dump_path.unlink()

    # Configure the algorithm
    population_size = 50
    genome_length = 55
    ga = GeneticAlgorithm(fitness_function)
    ga.generate_binary_population(size=population_size, genome_length=genome_length)

    # How many pairs of individuals should be picked to mate
    ga.number_of_pairs = 5

    # Selective pressure from interval [1.0, 2.0]
    # The lower the value, the less the fitness will play a role
    ga.selective_pressure = 1.5
    ga.mutation_rate = 0.1

    # If two parents have the same genotype, ignore them and generate TWO random parents.
    # This helps prevent premature convergence.
    ga.allow_random_parent = True  # default True
    # Use single point crossover instead of uniform crossover
    ga.single_point_cross_over = False  # default False

    # Run 50 iterations of the algorithm
    ga.run(50)

    best_genome, best_fitness = ga.get_best_genome()

    print("BEST FITNESS IS")
    print(best_fitness)
    print("BEST CHROMOSOME IS")
    print(best_genome)
    print("Its decoded value is:")
    print(f"outlier_rejection_quantile = {decode_function(best_genome[0:10])}")
    print(f"max_number_landmarks_near_bin = {decode_function(best_genome[11:22]) * 1000}")
    print(f"max_number_landmarks_middle_bin = {decode_function(best_genome[23:33]) * 1000}")
    print(f"max_number_landmarks_far_bin = {decode_function(best_genome[34:44]) * 1000}")
    print(f"shrubbery_weight = {decode_function(best_genome[45:55])}")

    # Optionally, you can inspect the full population and fitness vector:
    population = ga.population
    fitness_vector = ga.get_fitness_vector()

if __name__ == "__main__":
    main()
