#!/usr/bin/env python3
from pathlib import Path

# Define the file path
file_path = Path('/tmp/poses_dump.txt')

# Read the file and count lines
try:
    content = file_path.read_text(encoding='utf-8')
except Exception as e:
    print(f"Error reading file: {e}")
    exit(1)

lines = content.splitlines()
num_lines = len(lines)
print(f"Initial number of lines: {num_lines}")

# If there are more than 268 lines, rewrite the file to only include the first 268 lines.
if num_lines > 268:
    print("Modifying poses dump to 268 poses")
    new_content = "\n".join(lines[:268]) + "\n"
    try:
        file_path.write_text(new_content, encoding='utf-8')
    except Exception as e:
        print(f"Error writing file: {e}")
        exit(1)

# Read back the file to verify the change
try:
    updated_lines = file_path.read_text(encoding='utf-8').splitlines()
    print(f"Final number of lines: {len(updated_lines)}")
except Exception as e:
    print(f"Error reading file after modification: {e}")
    exit(1)
