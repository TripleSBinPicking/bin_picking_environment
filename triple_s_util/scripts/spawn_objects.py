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
                'xyz': [0.357636, -0.229424, 1.022993],
                'rpy': [1.559174, 0, 0.304139]
            },
            {
                'xyz': [0.091557, -0.297904, 1.127250],
                'rpy': [1.610274, -0.046245, -2.277866]
            }
        ]
    },
    {
        'model_name': 'Pen',
        'model_path': 'package://triple_s_util/meshes/pen/model.sdf',
        'poses': [
            {
                'xyz': [-0.23182, -0.358985, 1.019873],
                'rpy': [-1.605724, 0, 0]
            },
            {
                'xyz': [0.277022, -0.327723, 1.139273],
                'rpy': [1.625477, 0.022515, -1.887787]
            }
        ]
    },
    {
        'model_name': 'Peppermint',
        'model_path': 'package://triple_s_util/meshes/peppermint/model.sdf',
        'poses': [
            {
                'xyz': [-0.069821, -0.245784, 1.058283],
                'rpy': [-0.571517, 0.604930, -1.015215]
            },
            {
                'xyz': [0.173382, -0.312944, 1.077713],
                'rpy': [-2.452620, 0, 0]
            },
        ]
    },
    {
        'model_name': 'USB',
        'model_path': 'package://triple_s_util/meshes/usb_stick/model.sdf',
        'poses': [
            {
                'xyz': [0.225591, -0.405132, 1.026590],
                'rpy': [0, 0, 0]
            },
            {
                'xyz': [0.139838, -0.254065, 1.117130],
                'rpy': [-1.537831, 0, 0]
            },
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