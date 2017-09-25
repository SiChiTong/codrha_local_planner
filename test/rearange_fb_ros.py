#!/usr/bin/env python
import yaml
# INITIALIZATION
prefix = '../log/'
filename = 'fb_log_CODRHAPlannerROS'

start_time = -1
odom_ts = {
    'start_time': -1,
    'stamp': [],
    'pose': {
        'position': {
            'x': [],
            'y': [],
            'z': []
        },
        'orientation': {
            'x': [],
            'y': [],
            'z': [],
            'w': []
        }
    },
    'twist': {
        'linear': {
            'x': [],
            'y': [],
            'z': []
        },
        'angular': {
            'x': [],
            'y': [],
            'z': []
        }
    }
}

# GETTING DATA

with open(prefix + filename + '.yaml', 'r') as documents:
    for document in yaml.load_all(documents):

        if document == None:
            break

        # print('.')
        # sys.stdout.write('.')
        # print document['header']['stamp']['secs'] + document['header'][
        # 'stamp']['nsecs'] * 1e-9
        if start_time == -1:
            # toc_sec + toc_usec*1e-6
            start_time = document['header']['stamp']['secs'] + document[
                'header']['stamp']['nsecs'] * 1e-9
            odom_ts['start_time'] = start_time

        odom_ts['stamp'].append(document['header']['stamp']['secs'] + document[
            'header']['stamp']['nsecs'] * 1e-9 - start_time)

        odom_ts['pose']['position']['x'].append(document['pose']['pose'][
            'position']['x'])
        odom_ts['pose']['position']['y'].append(document['pose']['pose'][
            'position']['y'])
        odom_ts['pose']['position']['z'].append(document['pose']['pose'][
            'position']['z'])

        odom_ts['pose']['orientation']['x'].append(document['pose']['pose'][
            'orientation']['x'])
        odom_ts['pose']['orientation']['y'].append(document['pose']['pose'][
            'orientation']['y'])
        odom_ts['pose']['orientation']['z'].append(document['pose']['pose'][
            'orientation']['z'])
        odom_ts['pose']['orientation']['w'].append(document['pose']['pose'][
            'orientation']['w'])

        odom_ts['twist']['linear']['x'].append(document['twist']['twist'][
            'linear']['x'])
        odom_ts['twist']['linear']['y'].append(document['twist']['twist'][
            'linear']['y'])
        odom_ts['twist']['linear']['z'].append(document['twist']['twist'][
            'linear']['z'])

        odom_ts['twist']['angular']['x'].append(document['twist']['twist'][
            'angular']['x'])
        odom_ts['twist']['angular']['y'].append(document['twist']['twist'][
            'angular']['y'])
        odom_ts['twist']['angular']['z'].append(document['twist']['twist'][
            'angular']['z'])

with open(prefix + 'rearanged_' + filename + '.yaml', 'w') as stream:
    yaml.dump(odom_ts, stream)
