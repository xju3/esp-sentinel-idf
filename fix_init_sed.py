import sys

with open('main/init.c', 'r', encoding='utf-8') as f:
    content = f.read()

# Let's see what the sed command actually did
print(content[600:1000]) # Print a snippet around where the function was

