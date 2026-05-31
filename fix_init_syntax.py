import sys

with open('main/init.c', 'r', encoding='utf-8') as f:
    lines = f.readlines()

with open('main/init.c', 'w', encoding='utf-8') as f:
    for i, line in enumerate(lines):
        # The sed command missed these two lines because they were after the first closing brace inside the function
        if i == 77 or i == 78:
            continue
        f.write(line)
