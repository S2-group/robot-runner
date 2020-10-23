from typing import List

class ExperimentTreatmentModel:
    __column_name:   str       = None
    __column_values: List[str] = None

    def __init__(self, column_name: str, column_values: List[str]):
        self.__column_name = column_name
        self.__column_values = column_values

    def get_column_name(self) -> str:
        return self.__column_name

    def get_column_values(self) -> List[str]:
        return self.__column_values