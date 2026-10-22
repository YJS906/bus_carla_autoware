#!/usr/bin/env python3
"""Run the installed converter with bus maps in an isolated build/test ROS domain."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile

import yaml

share = Path('/opt/carla_overlay/share/autoware_carla_interface')
parameters = yaml.safe_load((share/'config/raw_vehicle_cmd_converter.param.yaml').read_text())
params = parameters['/**']['ros__parameters']
for kind in ('accel', 'brake', 'steer'):
    params['csv_path_'+kind+'_map'] = str(share/(kind+'_map.csv'))
with tempfile.TemporaryDirectory() as directory:
    parameter_file = Path(directory)/'converter.yaml'
    parameter_file.write_text(yaml.safe_dump(parameters))
    with (Path(directory)/'converter.log').open('w+') as log:
        command = ['/opt/autoware/autoware_raw_vehicle_cmd_converter/lib/'
                   'autoware_raw_vehicle_cmd_converter/autoware_raw_vehicle_cmd_converter_node',
                   '--ros-args', '--params-file', str(parameter_file)]
        environment = dict(os.environ, ROS_DOMAIN_ID='211')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=environment)
        try:
            process.wait(timeout=4)
            raise RuntimeError('Converter exited during startup: '+str(process.returncode))
        except subprocess.TimeoutExpired:
            print('Bus converter remained alive with installed pedal maps.')
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.seek(0)
            print(log.read())
