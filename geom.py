from numpy import cos, sin, sqrt
from math import isclose

class Quat:
    def __init__(self, q0, q1, q2, q3):
        """Quaternion

        :param q0: real part
        :param q1: imaginary/vector part i
        :param q2: imaginary/vector part j
        :param q3: imaginary/vector part k
        :return: Quaternion
        :rtype: Quaternion
        """
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def __mul__(p, q):
        q0, q1, q2, q3 = q.q0, q.q1, q.q2, q.q3
        p0, p1, p2, p3 = p.q0, p.q1, p.q2, p.q3

        r0 = -p1 * q1 - p2 * q2 - p3 * q3 + p0 * q0
        r1 =  p1 * q0 + p2 * q3 - p3 * q2 + p0 * q1
        r2 = -p1 * q3 + p2 * q0 + p3 * q1 + p0 * q2
        r3 =  p1 * q2 - p2 * q1 + p3 * q0 + p0 * q3
        return Quat(r0, r1, r2, r3)

    
    def conj(self):
        return Quat(self.q0, -self.q1, -self.q2, -self.q3)
    
    def vector_part(self):
        return CartVec3(self.q1, self.q2, self.q3)
    
    def normalize(self):
        norm = self.norm()
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm
        return self
    
    def norm(self):
        return sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
    
    @staticmethod
    def get_rotation_quat_around(θ, u):
        q0 = cos(θ/2.)
        q1 = u.x * sin(θ/2.)
        q2 = u.y * sin(θ/2.)
        q3 = u.z * sin(θ/2.)
        return Quat(q0, q1, q2, q3)
    
    @staticmethod
    def get_rotation_quat_from_to(u, v):
        u, v = u.normalize(), v.normalize()
        q = Quat(0, u.x, u.y, u.z) * Quat(0, v.x, v.y, v.z)
        return Quat(1-q.q0, q.q1, q.q2, q.q3).normalize()
    
    def rotate(v, q):
        return (q * v.to_pure_quat() * q.conj()).vector_part()

class PolarVec3:

    def __init__(self, ρ, θ, ϕ):
        """Cartesian 3D Vector

        :param ρ: radial distance
        :param θ: polar angle
        :param ϕ: azimuthal angle
        :return: 3D Vector
        :rtype: PolarVec3
        """
        self.ρ = ρ
        self.θ = θ
        self.ϕ = ϕ
        
    def to_cart(self):
        x = self.ρ * sin(self.θ) * cos(self.ϕ)
        y = self.ρ * sin(self.θ) * sin(self.ϕ)
        z = self.ρ * cos(self.θ)
        return CartVec3(x, y, z)

class CartVec3:
    def __init__(self, x, y, z):
        """Cartesian 3D Vector

        :param x: coordinate x
        :param y: coordinate y
        :param z: coordinate z
        :return: 3D Vector
        :rtype: CartVec3
        """
        self.x = x
        self.y = y
        self.z = z

    def __add__(a, b):
        return CartVec3(a.x + b.x, a.y + b.y, a.z + b.z)

    def __eq__(p, q):
        return isclose(p.x, q.x) and isclose(p.y, q.y) and isclose(p.z, q.z)
        
    def normalize(self):
        norm = self.norm()
        self.x /= norm
        self.y /= norm
        self.z /= norm
        return self
    
    def norm(self):
        return sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def to_pure_quat(self):
        return Quat(0, self.x, self.y, self.z)
    
    @staticmethod
    def dot(a, b):
        return a.x * b.x + a.y * b.y + a.z + b.z