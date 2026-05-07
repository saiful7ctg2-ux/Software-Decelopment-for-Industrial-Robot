import json
from ..postypes.SixDPos import SixDPos
from ..postypes.configuration import configuration


class OpMode:
    CFG_2_POS = 0
    POS_2_CFG = 1
    PTP = 2
    PTPSYNC = 3
    LIN = 4
    CIRC = 5


class JsonHandler:
    def __init__(self, json_string):
        self.m_json_value = json.loads(json_string)
        self.m_op_mode = self._parse_op_mode()
        self.m_data = self.m_json_value.get("data", [])
        # NEW: Parse IK method from JSON
        self.m_ik_method = self.m_json_value.get("ik_method", "both")  # default to both

    def _parse_op_mode(self):
        op_code = self.m_json_value.get("op", -1)
        if op_code == 0:
            return OpMode.CFG_2_POS
        elif op_code == 1:
            return OpMode.POS_2_CFG
        elif op_code == 2:
            return OpMode.PTP
        elif op_code == 3:
            return OpMode.PTPSYNC
        elif op_code == 4:
            return OpMode.LIN
        elif op_code == 5:
            return OpMode.CIRC
        else:
            return None

    def get_op_mode(self):
        return self.m_op_mode

    def get_data(self):
        return self.m_data

    # NEW: Get IK method
    def get_ik_method(self):
        
        return self.m_ik_method

    def get_circ_data(self):
        data = self.m_data
        if len(data) >= 4:
            start_cfg_data = data[0]
            end_cfg_data = data[1]
            aux_cfg_data = data[2]
            clockwise = data[3]

            if isinstance(clockwise, str):
                clockwise = clockwise.lower() == "true"

            return start_cfg_data, end_cfg_data, aux_cfg_data, clockwise
        return None

    def get_json_string(self, data, ik_method=None):
        
        if isinstance(data, SixDPos):
            self.m_op_mode = OpMode.POS_2_CFG
            data = [data]
        elif isinstance(data, configuration):
            self.m_op_mode = OpMode.CFG_2_POS
            data = [data]
        elif isinstance(data, list) and all(isinstance(cfg, configuration) for cfg in data):
            self.m_op_mode = OpMode.CFG_2_POS

        value = {
            "op": self.m_op_mode,
            "data": [item.serialize_to_json() for item in data]
        }
        
        # NEW: Include IK method in response if provided
        if ik_method:
            value["ik_method"] = ik_method
        
        return json.dumps(value)