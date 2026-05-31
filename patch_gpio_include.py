import sys

files_to_patch = [
    'components/task/src/off_sleep_manager.c',
    'components/task/src/task_baseline.c',
    'components/peri/src/daq_iis3dwb.c'
]

for file_path in files_to_patch:
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    if '#include "driver/gpio.h"' not in content:
        content = '#include "driver/gpio.h"\\n' + content
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)

