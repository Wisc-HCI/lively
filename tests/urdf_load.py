from lively_tk import parse_config_data, Solver
import yaml
import json

config_file = 'nao.json'

with open(config_file) as handle:
    data = yaml.safe_load(handle)

config = parse_config_data(data)

config.urdf = data['urdf']

print(config.joints[0])

solver = Solver(config)

print(solver)