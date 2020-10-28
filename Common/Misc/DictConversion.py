import inspect
from typing import Dict, List

def class_to_dict(obj):
    pr = {}
    for name in dir(obj):
        value = getattr(obj, name)
        if not name.startswith('__') and not inspect.ismethod(value):
            pr[name] = value
    return pr

def pop_from_each_dict_in_list(list_of_dicts: List[Dict], to_be_popped_key: str, default = None) -> List[Dict]:
    for dict in list_of_dicts:
        dict.pop(to_be_popped_key, default)
    
    return list_of_dicts