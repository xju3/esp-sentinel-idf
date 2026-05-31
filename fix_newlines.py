import sys
import re

files_to_fix = [
    'components/task/src/off_sleep_manager.c',
    'components/task/src/task_baseline.c',
    'components/peri/src/daq_iis3dwb.c'
]

for file_path in files_to_fix:
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Python script wrote literal '\n' instead of actual newlines. Fixing that.
    content = content.replace('\\n', '\n')
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)

