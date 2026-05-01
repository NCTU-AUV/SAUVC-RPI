from .vector_3d import Vector3D


class Quaternion:

    def __init__(self, real_part, i_part, j_part, k_part):
        self.__real_part = real_part
        self.__vector_part = Vector3D(i_part, j_part, k_part)

    def __mul__(self, other):
        real_part = self.__real_part * other.__real_part - self.__vector_part.dot_product(other.__vector_part)
        vector_part = (
            self.__vector_part.scalar_multiple(other.__real_part) +
            other.__vector_part.scalar_multiple(self.__real_part) +
            self.__vector_part.cross_product(other.__vector_part)
        )

        return Quaternion(real_part, vector_part.x, vector_part.y, vector_part.z)

    @property
    def inverse(self):
        denominator = self.__real_part**2 + self.__vector_part.dot_product(self.__vector_part)

        real_part = self.__real_part / denominator
        i_part = - self.__vector_part.x / denominator
        j_part = - self.__vector_part.y / denominator
        k_part = - self.__vector_part.z / denominator

        return Quaternion(real_part, i_part, j_part, k_part)

    @property
    def vector_part(self):
        return self.__vector_part
