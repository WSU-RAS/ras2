# For data loading
import yaml

file_location = "../Data/casas_map.yaml"

with open(file_location, 'r') as stream:
    try:
        data = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print(data)
