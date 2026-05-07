class SixDPos:
    X, Y, Z, A, B, C = range(6)

    def __init__(self, *args):
        if len(args) == 0:
            self.m_position = [0.0] * 6
        elif len(args) == 1 and isinstance(args[0], (list, tuple)) and len(args[0]) == 6:
            self.m_position = list(args[0])
        elif len(args) == 6:
            self.m_position = list(args)
        elif len(args) == 1 and isinstance(args[0], dict):
            json_dict = args[0]
            self.m_position = [
                json_dict.get("m_x", 0.0),
                json_dict.get("m_y", 0.0),
                json_dict.get("m_z", 0.0),
                json_dict.get("m_a", 0.0),
                json_dict.get("m_b", 0.0),
                json_dict.get("m_c", 0.0)
            ]
        else:
            raise ValueError("Invalid arguments")

    def set_position(self, _position):
        if len(_position) != 6:
            raise ValueError("Invalid position length, should be 6")
        self.m_position = _position

    def get_position(self):
        return self.m_position

    def __getitem__(self, index):
        return self.m_position[index]

    def serialize_to_json(self):
        json_pose = {
            "m_x": self.m_position[self.X],
            "m_y": self.m_position[self.Y],
            "m_z": self.m_position[self.Z],
            "m_a": self.m_position[self.A],
            "m_b": self.m_position[self.B],
            "m_c": self.m_position[self.C]
        }
        return json_pose

    def deserialize_from_json(self, json_value):
        if "m_x" in json_value:
            self.m_position[self.X] = json_value["m_x"]
        if "m_y" in json_value:
            self.m_position[self.Y] = json_value["m_y"]
        if "m_z" in json_value:
            self.m_position[self.Z] = json_value["m_z"]
        if "m_a" in json_value:
            self.m_position[self.A] = json_value["m_a"]
        if "m_b" in json_value:
            self.m_position[self.B] = json_value["m_b"]
        if "m_c" in json_value:
            self.m_position[self.C] = json_value["m_c"]