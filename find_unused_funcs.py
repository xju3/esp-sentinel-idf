import os
import re

c_files = []
h_files = []
for root, dirs, files in os.walk('components'):
    for f in files:
        if f.endswith('.c'): c_files.append(os.path.join(root, f))
        elif f.endswith('.h'): h_files.append(os.path.join(root, f))
for root, dirs, files in os.walk('main'):
    for f in files:
        if f.endswith('.c'): c_files.append(os.path.join(root, f))
        elif f.endswith('.h'): h_files.append(os.path.join(root, f))

# regex for C function declaration
func_re = re.compile(r'^\s*([a-zA-Z_][a-zA-Z0-9_\s\*]+)\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\([^)]*\)\s*;')

for h_file in h_files:
    with open(h_file, 'r') as f:
        content = f.read()
    
    for match in func_re.finditer(content):
        ret_type = match.group(1).strip()
        func_name = match.group(2)
        
        if func_name.startswith('esp_') or func_name == 'main': continue
        
        # Count usages
        usage_count = 0
        used_in = []
        for c_file in c_files:
            with open(c_file, 'r') as cf:
                if func_name in cf.read():
                    usage_count += 1
                    used_in.append(c_file)
        
        if usage_count <= 1:
            print(f"Unused or internal only: {func_name} (in {h_file}) used in {used_in}")

