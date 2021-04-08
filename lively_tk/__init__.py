from .lively_tk import *

CONFIG_FIELDS = {
        'axis_types', 'base_link_motion_bounds', 'ee_fixed_joints', 'static_environment',
        'fixed_frame', 'goals', 'modes', 'joint_limits', 'joint_names', 'joint_ordering',
        'joint_types', 'mode_control', 'mode_environment','nn_jointpoint', 'nn_main',
        'objectives', 'states', 'robot_link_radius', 'rot_offsets', 'starting_config',
        'urdf','velocity_limits', 'disp_offsets', 'displacements'
    }

ENVIRONMENT_SPEC_FIELDS = {'cuboids', 'spheres', 'pcs'}

NN_SPEC_FIELDS = {'coefs', 'intercepts', 'split_point'}

MODE_CONFIG_FIELDS = {'name', 'weights'}

GOAL_CONFIG_FIELDS = {'name', 'values'}

OBJECTIVE_SPEC_FIELDS = {'variant', 'tag', 'indices', 'scale', 'shape', 'frequency'}

CUBOID_FIELDS = {'name','x_halflength','y_halflength','z_halflength','rx','ry','rz','tx','ty','tz','is_dynamic','coordinate_frame'}

SPHERE_FIELDS = {'name','radius','tx','ty','tz','is_dynamic','coordinate_frame'}

GOAL_SPEC_FIELDS = {'scalar','vector','quaternion','pose'}

OBJECTIVE_INPUT_FIELDS = {'weight', 'scalar','vector','quaternion'}

CONVERSIONS_TO_CONFIG = {
 'states':lambda states: [tuple(state) for state in states],
 'starting_config':lambda starting_config: tuple(starting_config)
}

def handle_keys(data,field_set):
    cleaned = {}
    for key,value in data.items():
        if key in field_set:
            try:
                cleaned[key] = CONVERSIONS_TO_CONFIG[key](value)
            except:
                cleaned[key] = value
    return cleaned

def parse_config_data(data:dict) -> Config:
    def parse_data(item):
        obj = None
        if isinstance(item,(int,float,str,bool)):
            obj = item
        elif isinstance(item,list):
            obj = [parse_data(o) for o in item]
        elif isinstance(item,dict):
            content = {key:parse_data(value) for key,value in item.items()}
            if set(content.keys()).issuperset(CONFIG_FIELDS):
                obj = Config(**handle_keys(content,CONFIG_FIELDS))
            elif set(content.keys()).issuperset(ENVIRONMENT_SPEC_FIELDS):
                obj = EnvironmentSpec(**{key:value for key,value in content.items() if key in ENVIRONMENT_SPEC_FIELDS })
            elif set(content.keys()).issuperset(NN_SPEC_FIELDS):
                obj = NNSpec(**{key:value for key,value in content.items() if key in NN_SPEC_FIELDS })
            elif set(content.keys()).issuperset(MODE_CONFIG_FIELDS):
                obj = ModeConfig(**{key:value for key,value in content.items() if key in MODE_CONFIG_FIELDS })
            elif set(content.keys()).issuperset(GOAL_CONFIG_FIELDS):
                obj = GoalConfig(**{key:value for key,value in content.items() if key in GOAL_CONFIG_FIELDS })
            elif set(content.keys()).issuperset(CUBOID_FIELDS):
                obj = Cuboid(**{key:value for key,value in content.items() if key in CUBOID_FIELDS })
            elif set(content.keys()).issuperset(SPHERE_FIELDS):
                obj = Sphere(**{key:value for key,value in content.items() if key in SPHERE_FIELDS })
            elif 'variant' in content.keys() and 'indices' in content.keys() and set(content.keys()).issubset(OBJECTIVE_SPEC_FIELDS):
                obj = ObjectiveSpec(**{key:value for key,value in content.items() if key in OBJECTIVE_SPEC_FIELDS })
            elif 'weight' in content.keys() and set(content.keys()).issubset(OBJECTIVE_INPUT_FIELDS):
                obj = ObjectiveInput(**content)
            elif set(content.keys()).issubset(GOAL_SPEC_FIELDS):
                obj = GoalSpec(**content)
            else:
                print("Could not identify lookup with keys {0}".format(content.keys()))
        return obj
    return parse_data(data)

def export_config_data(config:Config) -> dict:
    def parse_object(item):
        data = {}
        if isinstance(item,(int,float,str,bool)):
            data = item
        elif isinstance(item,list):
            data = [parse_object(o) for o in item]
        elif isinstance(item,Config):
            for field in CONFIG_FIELDS:
                data[field] = parse_object(getattr(item,field))
        elif isinstance(item,EnvironmentSpec):
            for field in ENVIRONMENT_SPEC_FIELDS:
                data[field] = parse_object(getattr(item,field))
        elif isinstance(item,NNSpec):
            for field in NN_SPEC_FIELDS:
                data[field] = parse_object(getattr(item,field))
        elif isinstance(item,ModeConfig):
            for field in MODE_CONFIG_FIELDS:
                data[field] = parse_object(getattr(item,field))
        elif isinstance(item,GoalConfig):
            for field in GOAL_CONFIG_FIELDS:
                data[field] = parse_object(getattr(item,field))
        elif isinstance(item,ObjectiveSpec):
            for field in OBJECTIVE_SPEC_FIELDS:
                data[field] = parse_object(getattr(item,field))
        elif isinstance(item,GoalSpec):
            for field in GOAL_SPEC_FIELDS:
                value = parse_object(getattr(item,field))
                if value:
                    data[field] = value
        elif isinstance(item,ObjectiveInput):
            for field in OBJECTIVE_INPUT_FIELDS:
                value = parse_object(getattr(item,field))
                if value:
                    data[field] = value
        return data

    return parse_object(config)
