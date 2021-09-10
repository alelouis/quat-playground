from geom import CartVec3, PolarVec3, Quat
from numpy import pi, random

if __name__ == "__main__":
    u = PolarVec3(1.0, random.rand(1)*pi, random.rand(1)*2*pi).to_cart()
    v = CartVec3(random.rand(1), random.rand(1), random.rand(1))
    q = Quat.get_rotation_quat_from_to(u, v)
    u_ = Quat.rotate(u, q)
    assert u_ == v
