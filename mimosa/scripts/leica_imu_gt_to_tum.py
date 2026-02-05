#!/usr/bin/env python3
import sys
import csv

def convert_csv(input_path, output_path):
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        reader = csv.DictReader(infile)
        for row in reader:
            outfile.write(f"{row['timestamp']} {row['x']} {row['y']} {row['z']} {row['qx']} {row['qy']} {row['qz']} {row['qw']}\n")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert.py <input.csv> <output.tum>")
        sys.exit(1)
    convert_csv(sys.argv[1], sys.argv[2])