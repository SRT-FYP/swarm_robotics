# preprocess_nav2_yaml.py
import sys
try:
    namespace = sys.argv[1]
except:
    namespace = ""
    
with open('../params/nav2_multirobot_params.yaml') as f:
    content = f.read()
with open('../params/temp_nav2_params_namespaced.yaml', 'w') as f:
    f.write(content.replace('<robot_namespace>', namespace))