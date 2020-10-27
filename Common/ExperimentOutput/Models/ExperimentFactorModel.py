from typing import List

class ExperimentFactorModel:
    __factor_name:   str    = None
    __treatments: List[str] = None

    def __init__(self, factor_name: str, treatments: List[str]):
        self.__factor_name = factor_name
        self.__treatments = treatments

    def get_factor_name(self) -> str:
        return self.__factor_name

    def get_treatments(self) -> List[str]:
        return self.__treatments