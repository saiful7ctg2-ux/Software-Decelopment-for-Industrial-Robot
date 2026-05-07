NUM_JOINTS = 6


class configuration:
    def __init__(self, value=None):
        if value is None:
            self.joints = [0.0] * NUM_JOINTS
        elif isinstance(value, list) and len(value) == NUM_JOINTS:
            self.joints = list(value)
        elif isinstance(value, dict):
            self.deserialize_from_json(value)
        else:
            raise ValueError("Invalid argument type for Configuration")

    def set_configuration(self, joints):
        self.joints = joints

    def __getitem__(self, index):
        return self.joints[index]

    def __setitem__(self, index, value):
        self.joints[index] = value

    def get_configuration(self):
        return self.joints

    def serialize_to_json(self):
        json_pose = {
            "j0": self.joints[0],
            "j1": self.joints[1],
            "j2": self.joints[2],
            "j3": self.joints[3],
            "j4": self.joints[4],
            "j5": self.joints[5]
        }
        return json_pose

    def deserialize_from_json(self, json_value):
        if 'j0' in json_value and 'j1' in json_value and 'j2' in json_value and 'j3' in json_value and 'j4' in json_value and 'j5' in json_value:
            self.set_configuration([
                float(json_value['j0']),
                float(json_value['j1']),
                float(json_value['j2']),
                float(json_value['j3']),
                float(json_value['j4']),
                float(json_value['j5'])
            ])
        else:
            # Handle the case where values cannot be converted to floats.
            # Can raise an error or handle it as per requirements.
            print("Joint values can not be converted to float")
            pass
