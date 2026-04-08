
class Vector3D:

    def __init__(self, x, y, z):
        self.__x = x
        self.__y = y
        self.__z = z

    def dot_product(self, vector):
        return self.__x * vector.__x + self.__y * vector.__y + self.__z * vector.__z

    def cross_product(self, vector):
        x = self.__y * vector.__z - vector.__y * self.__z
        y = -(self.__x * vector.__z - vector.__x * self.__z)
        z = self.__x * vector.__y - vector.__x * self.__y

        return Vector3D(x, y, z)

    def scalar_multiple(self, multiple):
        x = self.__x * multiple
        y = self.__y * multiple
        z = self.__z * multiple

        return Vector3D(x, y, z)

    def __add__(self, other):
        x = self.__x + other.__x
        y = self.__y + other.__y
        z = self.__z + other.__z

        return Vector3D(x, y, z)

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z
