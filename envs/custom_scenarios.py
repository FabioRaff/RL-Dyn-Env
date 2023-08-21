# Dynamic Lifted Obstacles Scenario

lifted_obst = {
    'obj_init_space': {
        'min': [1.15, 1.075],
        'max': [1.45, 1.175]
    },
    'target_init_space': {
        'min': [1.15, 0.35, 0.42],
        'max': [1.45, 0.45, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.451],
        'size': [0.025, 0.025, 0.05],
        'vel': {
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.85, 0.451],
        'site_size': [0.2, 0.025, 0.05]
    },
    'obstacle1': {
        'pos': [1.3, 0.6, 0.426],
        'size': [0.2, 0.025, 0.025],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.6, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [1.3, 0.6, 0.48],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.6, 0.48],
        'site_size': [0.2, 0.025, 0.025]
    }
}

scenarios = {
    'lifted_obst': lifted_obst
}
