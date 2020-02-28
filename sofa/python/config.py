'''
    general configuration for setup
    IABs are so named:
    Convention is top of head is facing the screen. Left is to your left ear as it would be if you were facing the screen

    Bottom IABs: {{neck_left, neck_right},{skull_left, skull_right}}
    Side   IABs: {{fore_left, chin_left}, {fore_right, chin_right}}
'''
import numpy as np

setuptype = {
        'MRI': False, # can only be one of MRI or LINAC but not both
        'LINAC': False,
        'Flexis': False
}

# open-loop control settings
thresholds =dict(patient_trans=None, # [508667.26419036+10000, 134780.90110167+10000, 189716.76200505+10000,]
                 patient_rot=[np.pi/2, np.pi/4, -np.pi], #yaw, pitch, roll
                 )

move_dist = (0, .40, 0)
growth_rate = 100  #was .05
max_pressure = 100 # was 15
