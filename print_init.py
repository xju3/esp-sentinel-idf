import sys

with open('main/init.c', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if 65 <= i <= 85:
        print(f"{i+1:3d}: {line}", end='')
