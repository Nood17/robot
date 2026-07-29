from dataclasses import dataclass, field
from enum import Enum
# 导入当前模块的工具函数
from .tools import identify_function


# 手动定义模型类
class PropertyType(Enum):
    BOOLEAN = "boolean"
    INTEGER = "integer"
    STRING = "string"


@dataclass
class Property:
    name: str
    type: PropertyType
    description: str = ""
    default_value: any = None
    required: bool = False
    min_value: int = None
    max_value: int = None


@dataclass
class PropertyList:
    properties: list[Property] = field(default_factory=list)


class IdentifyManager:
    def __init__(self):
        pass

    def init_tools(self, add_tool, PropertyList, Property, PropertyType):
        tool_props = PropertyList([])  # 无参数
        add_tool((
            "self.identify.trigger",  # 工具名称
            "触发身份识别，响应“我是谁”“认识我”指令",
            tool_props,
            identify_function  # 关联工具函数
        ))


_manager = None


def get_identify_manager():
    global _manager
    if _manager is None:
        _manager = IdentifyManager()
    return _manager
