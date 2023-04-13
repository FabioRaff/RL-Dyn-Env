# Dynamic Lifted Obstacles Scenario

lifted_obst = {
    'obj_init_space': {
        'min': [1.0, 1.1],
        'max': [1.4, 1.2]
    },
    'target_init_space': {
        'min': [1.0, 0.3, 0.42],
        'max': [1.4, 0.4, 0.42]
    },
    'obstacle0': {
        'pos': [1.2, 0.85, 0.451],
        'size': [0.025, 0.025, 0.05],
        'vel': {
            'min': 0.1,
            'max': 0.4
        },
        'dir': 0,
        'site_pos': [1.2, 0.85, 0.451],
        'site_size': [0.25, 0.025, 0.05]
    },
    'obstacle1': {
        'pos': [1.2, 0.6, 0.426],
        'size': [0.25, 0.025, 0.025],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.2, 0.6, 0.426],
        'site_size': [0.25, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [1.2, 0.6, 0.48],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
            'max': 0.4
        },
        'dir': 0,
        'site_pos': [1.2, 0.6, 0.48],
        'site_size': [0.25, 0.025, 0.025]
    }
}

scenarios = {
    'lifted_obst': lifted_obst
}
