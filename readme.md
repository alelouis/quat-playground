# quat-playground

Experimenting with Quaternions rotations from scratch for insights.  
Preliminary work for [stargazer](https://github.com/alelouis/stargazer) evolutions. *Not performance oriented.*

## Features
- Vec3 cartesian (x, y, z)
- Vec3 polar spherical (ρ, θ, ϕ)
- Quaternion (q0, q1, q2, q3)
- Quaternion rotation from axis and angle:  
```python
@staticmethod
def get_rotation_quat_around(θ, u):
    q0 = cos(θ/2.)
    q1 = u.x * sin(θ/2.)
    q2 = u.y * sin(θ/2.)
    q3 = u.z * sin(θ/2.)
    return Quat(q0, q1, q2, q3)
``` 
- Quaternion rotation from u to v vectors
```python
@staticmethod
def get_rotation_quat_from_to(u, v):
    u, v = u.normalize(), v.normalize()
    q = Quat(0, u.x, u.y, u.z) * Quat(0, v.x, v.y, v.z)
    return Quat(1-q.q0, q.q1, q.q2, q.q3).normalize()
```
