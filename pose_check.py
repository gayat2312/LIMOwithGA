#!/usr/bin/env python3
from pathlib import Path

# Define the file path
file_path = Path('/tmp/poses_dump.txt')

# Read file content and count lines
try:
    content = file_path.read_text(encoding='utf-8')
except Exception as e:
    print(f"Error reading file: {e}")
    exit(1)

lines = content.splitlines()
num_lines = len(lines)
print(f"Initial number of lines: {num_lines}")

# If there are more than 1098 lines, trim the file to 1098 lines
if num_lines > 1098:
    print("Modifying poses dump to 1098 poses")
    new_content = "\n".join(lines[:1098]) + "\n"
    try:
        file_path.write_text(new_content, encoding='utf-8')
    except Exception as e:
        print(f"Error writing file: {e}")
        exit(1)

# Read back and print the final number of lines
try:
    updated_lines = file_path.read_text(encoding='utf-8').splitlines()
    print(f"Final number of lines: {len(updated_lines)}")
except Exception as e:
    print(f"Error reading file after modification: {e}")
    exit(1)
