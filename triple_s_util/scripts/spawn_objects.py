#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         24-11-2020
# Description:  If run directly it will spawn in some objects in gazebo according to the configuration.
#               Otherwise it will just contain the methods to do this.
import rospy
from gazebo_msgs.srv import SpawnModelRequest, SpawnModel
from tf_conversions import transformations
import resource_retriever

MODELS_TO_SPAWN = [
    {
        'model_name': 'Beeropener',
        'model_path': 'package://triple_s_util/meshes/beer_opener/model.sdf',
        'poses': [
            {
                'xyz': [-0.074726, -0.3, 1.023151],
                'rpy': [0, 0, 3.1415]
            },
            {
                'xyz': [0.162720, -0.331484, 1.018132],
                'rpy': [0.174100, 0.017003, 0]
            },
            {
                'xyz': [0.306033, -0.318942, 1.013760],
                'rpy': [-3.011879, 0.022726, -1.7432277]
            },
            {
                'xyz': [0.382137, -0.382137, 1.007056],
                'rpy': [0, 0, -0.324036]
            }
        ]
    },
    {
        'model_name': 'Pen',
        'model_path': 'package://triple_s_util/meshes/pen/model.sdf',
        'poses': [
            {
                'xyz': [0.0592283, -0.2342293, 1.037362],
                'rpy': [0.742413, 0.105806, 0.096598]
            },
            {
                'xyz': [0.252880, -0.393531, 1.014143],
                'rpy': [-2.192864, 0.0, 1.582755]
            }
        ]
    },
    {
        'model_name': 'Peppermint',
        'model_path': 'package://triple_s_util/meshes/peppermint/model.sdf',
        'poses': [
            {
                'xyz': [0.154504, -0.292099, 1.054356],
                'rpy': [-3.141592, 1.005141, 1.519735]
            },
            {
                'xyz': [0.050817, -0.368547, 1.009596],
                'rpy': [0, 0, -0.456186]
            }
        ]
    }
]

def createSpawnMessage(
    model_name='',
    model_path='',
    xyz=[0, 0, 0],
    rpy=[0, 0, 0],
    reference_frame='world'
):
    """
    Create a gazebo_msgs.msg.SpawnModel request message from the parameters.

    model_name -- Name of the model in the simulation
    model_path -- Path to the sdf file of the model
    xyz -- array of length 3 with the xyz coordinates
    rpy -- array of length 3 with the rpy rotations. These are converted to a quaternion
    reference_frame -- the reference frame of the coordinates

    returns -- SpawnModelRequest instance
    """
    request = SpawnModelRequest()
    request.model_name = model_name
    
    file = open(resource_retriever.get_filename(model_path, use_protocol=False))
    request.model_xml = file.read()
    file.close()

    request.initial_pose.position.x = xyz[0]
    request.initial_pose.position.y = xyz[1]
    request.initial_pose.position.z = xyz[2]

    quaternion = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

    request.initial_pose.orientation.x = quaternion[0]
    request.initial_pose.orientation.y = quaternion[1]
    request.initial_pose.orientation.z = quaternion[2]
    request.initial_pose.orientation.w = quaternion[3]
    request.reference_frame = reference_frame

    return request


def spawnModels(service, config):
    """
    Spawn models in gazebo.

    service -- The service proxy to the gazebo spawn service topic
    config -- The objects to spawn in an array. The array items should have the following format:
                  {
                      'model_name': 'Name of the model in the simulation',
                      'model_path': 'package://path/to/sdf',
                      'poses': [
                          {
                              'xyz': [0, 0, 0],
                              'rpy': [0, 0, 0]
                          }
                      ]
                  }  
              note that poses is also an array an that multiple instances of the object can be spawned
              by adding more poses. If this is done the model name will be concatenated with a number.
    """
    total_count = 0
    for model_settings in config:
        model_count = 0
        for pose in model_settings['poses']:
            message = createSpawnMessage(
                model_name=model_settings['model_name'] + str(model_count),
                model_path=model_settings['model_path'],
                xyz=pose['xyz'],
                rpy=pose['rpy']
            )
            model_count += 1
            
            result = service(message)

            if not result.success:
                rospy.logwarn('Could not spawn object: %s' % model_settings['model_name'])
                rospy.logwarn(result.status_message)
        
        total_count += model_count
    
    rospy.loginfo('Spawned %d models' % total_count)       

if __name__ == '__main__':
    rospy.init_node('spawn_models', anonymous=True)
    service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawnModels(service, MODELS_TO_SPAWN)