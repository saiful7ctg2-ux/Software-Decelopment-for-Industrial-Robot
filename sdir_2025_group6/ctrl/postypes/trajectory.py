class trajectory:
    def __init__(self):
        self.m_configs = []

    def __getitem__(self, index):
        return self.m_configs[index]

    def get_configuration(self, index):
        return self.m_configs[index]

    #@staticmethod
    def get_all_configuration(self):
        return self.m_configs

    def __setitem__(self, index, config):
        self.m_configs[index] = config

    def add_configuration(self, config):
        self.m_configs.append(config)

    def set_trajectory(self, _trajectory):
        self.m_configs = _trajectory

    def __eq__(self, other):
        if not isinstance(other, trajectory):
            return False
        return self.m_configs == other.m_configs