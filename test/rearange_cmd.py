#!/usr/bin/env python
import yaml
# INITIALIZATION
prefix = '../log/'

for i in range(5, 6):

    filename = 'cmmd_log_v' + str(i) + '_planner'

    start_time = -1
    cmd_ts = {
        'start_time': -1,
        'stamp': [],
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

        all_docs = yaml.load_all(documents)

        for document in all_docs:

            if document == None:
                break

            if document['header']['seq'] == 0:
                start_time = document['header']['stamp']['secs'] + document[
                    'header']['stamp']['nsecs'] * 1e-9
                cmd_ts['start_time'] = start_time
                break

    with open(prefix + filename + '.yaml', 'r') as documents:

        all_docs = yaml.load_all(documents)

        for document in all_docs:

            if document == None:
                break

            if start_time == -1:
                # toc_sec + toc_usec*1e-6

                start_time = document['header']['stamp']['secs'] + document[
                    'header']['stamp']['nsecs'] * 1e-9
                cmd_ts['start_time'] = start_time

            cmd_ts['stamp'].append(document['header']['stamp'][
                'secs'] + document['header']['stamp']['nsecs'] * 1e-9 -
                                   start_time)

            # cmd_ts['pose']['position']['x'].append(document['pose']['pose'][
            #     'position']['x'])
            # cmd_ts['pose']['position']['y'].append(document['pose']['pose'][
            #     'position']['y'])
            # cmd_ts['pose']['position']['z'].append(document['pose']['pose'][
            #     'position']['z'])
            #
            # cmd_ts['pose']['orientation']['x'].append(document['pose']['pose'][
            #     'orientation']['x'])
            # cmd_ts['pose']['orientation']['y'].append(document['pose']['pose'][
            #     'orientation']['y'])
            # cmd_ts['pose']['orientation']['z'].append(document['pose']['pose'][
            #     'orientation']['z'])
            # cmd_ts['pose']['orientation']['w'].append(document['pose']['pose'][
            #     'orientation']['w'])

            cmd_ts['twist']['linear']['x'].append(document['twist']['twist'][
                'linear']['x'])
            cmd_ts['twist']['linear']['y'].append(document['twist']['twist'][
                'linear']['y'])
            cmd_ts['twist']['linear']['z'].append(document['twist']['twist'][
                'linear']['z'])

            cmd_ts['twist']['angular']['x'].append(document['twist']['twist'][
                'angular']['x'])
            cmd_ts['twist']['angular']['y'].append(document['twist']['twist'][
                'angular']['y'])
            cmd_ts['twist']['angular']['z'].append(document['twist']['twist'][
                'angular']['z'])

    with open(prefix + 'rearanged_' + filename + '.yaml', 'w') as stream:
        yaml.dump(cmd_ts, stream)
