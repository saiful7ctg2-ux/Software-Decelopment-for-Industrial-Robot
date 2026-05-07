import math


class TMatrix:
    # TODO: Implement homogeneous matrix
    def __init__(self, *args):
        self.m_transformation = [[0] * 4 for _ in range(4)]

        if len(args) == 6:
            # Constructor with 6 individual arguments
            _trans_x, _trans_y, _trans_z, _rot_x, _rot_y, _rot_z = args
            self.m_transformation[0][0] = math.cos(_rot_x) * math.cos(_rot_y)

        else:
            raise ValueError("Invalid number of arguments")

    def get_matrix(self):
        return [row[:] for row in self.m_transformation]