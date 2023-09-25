# Dynamic Obstacles Scenarios

lifted_obst = {
    'obj_init_space': {
        'min': [1.2, 1.0],
        'max': [1.4, 1.05]
    },
    'target_init_space': {
        'min': [1.2, 0.45, 0.42],
        'max': [1.4, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.46],
        'size': [0.025, 0.025, 0.05],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.85, 0.46],
        'site_size': [0.2, 0.025, 0.05]
    },
    'obstacle1': {
        'pos': [1.3, 0.65, 0.43],
        'size': [0.2, 0.025, 0.025],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.43],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [1.3, 0.65, 0.49],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.49],
        'site_size': [0.2, 0.025, 0.025]
    }
}

dyn_sqr_obst = {
    'obj_init_space': {
        'min': [1.2, 1.0],
        'max': [1.4, 1.05]
    },
    'target_init_space': {
        'min': [1.2, 0.45, 0.42],
        'max': [1.4, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.426],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.85, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle1': {
        'pos': [1.3, 0.65, 0.426],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [0.8, 0.6, 0.42],
        'size': [0.02, 0.02, 0.02],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [0.8, 0.6, 0.42],
        'site_size': [0.02, 0.02, 0.02]
    }
}

dyn_obst_v1 = {
    'obj_init_space': {
        'min': [1.2, 1.0],
        'max': [1.4, 1.05]
    },
    'target_init_space': {
        'min': [1.2, 0.45, 0.42],
        'max': [1.4, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.426],
        'size': [0.06, 0.025, 0.025],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.85, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle1': {
        'pos': [1.3, 0.65, 0.426],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [0.8, 0.6, 0.42],
        'size': [0.02, 0.02, 0.02],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [0.8, 0.6, 0.42],
        'site_size': [0.02, 0.02, 0.02]
    }
}

dyn_obst_v2 = {
    'obj_init_space': {
        'min': [1.2, 1.0],
        'max': [1.4, 1.05]
    },
    'target_init_space': {
        'min': [1.2, 0.45, 0.42],
        'max': [1.4, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.426],
        'size': [0.1, 0.025, 0.025],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.85, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle1': {
        'pos': [1.3, 0.65, 0.426],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [0.8, 0.6, 0.42],
        'size': [0.02, 0.02, 0.02],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [0.8, 0.6, 0.42],
        'site_size': [0.02, 0.02, 0.02]
    }
}

sim2real = {
    'obj_init_space': {
        'min': [1.2, 1.0],
        'max': [1.4, 1.05]
    },
    'target_init_space': {
        'min': [1.2, 0.45, 0.42],
        'max': [1.4, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.75, 0.49],
        'size': [0.02, 0.042, 0.035],
        'vel': {
            'min': 0.01,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.75, 0.451],
        'site_size': [0.2, 0.042, 0.035]
    },
    'obstacle1': {
        'pos': [1.3, 0.75, 0.41],
        'size': [0.2, 0.02, 0.005],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.75, 0.41],
        'site_size': [0.2, 0.02, 0.005]
    },
    'obstacle2': {
        'pos': [0.8, 0.6, 0.42],
        'size': [0.02, 0.02, 0.02],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [0.8, 0.6, 0.42],
        'site_size': [0.02, 0.02, 0.02]
    }
}

scenarios = {
    'lifted_obst': lifted_obst,
    'dyn_sqr_obst': dyn_sqr_obst,
    'dyn_obst_v1': dyn_obst_v1,
    'dyn_obst_v2': dyn_obst_v2,
    'sim2real': sim2real
}
