# Dynamic Obstacles Scenarios

lifted_obst = {
    'obj_init_space': {
        'min': [1.15, 1.025],
        'max': [1.45, 1.125]
    },
    'target_init_space': {
        'min': [1.15, 0.4, 0.42],
        'max': [1.45, 0.5, 0.42]
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
        'pos': [1.3, 0.65, 0.426],
        'size': [0.2, 0.025, 0.025],
        'vel': {
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [1.3, 0.65, 0.48],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.48],
        'site_size': [0.2, 0.025, 0.025]
    }
}

dyn_sqr_obst = {
    'obj_init_space': {
        'min': [1.15, 1.025],
        'max': [1.45, 1.125]
    },
    'target_init_space': {
        'min': [1.15, 0.4, 0.42],
        'max': [1.45, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.426],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
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
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [0.01, 0.01, 0.01],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [0.01, 0.01, 0.01],
        'site_size': [0.025, 0.025, 0.025]
    }
}

dyn_obst_v1 = {
    'obj_init_space': {
        'min': [1.15, 1.025],
        'max': [1.45, 1.125]
    },
    'target_init_space': {
        'min': [1.15, 0.4, 0.42],
        'max': [1.45, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.426],
        'size': [0.1, 0.025, 0.025],
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
            'min': 0.,
            'max': 0.
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [0.01, 0.01, 0.01],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [0.01, 0.01, 0.01],
        'site_size': [0.025, 0.025, 0.025]
    }
}

dyn_obst_v2 = {
    'obj_init_space': {
        'min': [1.15, 1.025],
        'max': [1.45, 1.125]
    },
    'target_init_space': {
        'min': [1.15, 0.4, 0.42],
        'max': [1.45, 0.5, 0.42]
    },
    'obstacle0': {
        'pos': [1.3, 0.85, 0.426],
        'size': [0.1, 0.025, 0.025],
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
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [1.3, 0.65, 0.426],
        'site_size': [0.2, 0.025, 0.025]
    },
    'obstacle2': {
        'pos': [0.01, 0.01, 0.01],
        'size': [0.025, 0.025, 0.025],
        'vel': {
            'min': 0.1,
            'max': 0.1
        },
        'dir': 0,
        'site_pos': [0.01, 0.01, 0.01],
        'site_size': [0.025, 0.025, 0.025]
    }
}

scenarios = {
    'lifted_obst': lifted_obst,
    'dyn_sqr_obst': dyn_sqr_obst,
    'dyn_obst_v1': dyn_obst_v1,
    'dyn_obst_v2': dyn_obst_v2
}
