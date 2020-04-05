using LinearAlgebra
using StaticArrays
import Base.*

Rx(a) = @SMatrix [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]
Ry(a) = @SMatrix [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]
Rz(a) = @SMatrix [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]

function quaternion2matrix(q)  
  q0, q1, q2, q3 = q
  mat = @SMatrix[q0^2+q1^2-q2^2-q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);
                 2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
                 2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) q0^2-q1^2-q2^2+q3^2]
  return mat
end

function rpy2quaternion(r, p, y)
  qw = cos(r/2) * cos(p/2) * cos(y/2) + sin(r/2) * sin(p/2) * sin(y/2)
  qx = sin(r/2) * cos(p/2) * cos(y/2) - cos(r/2) * sin(p/2) * sin(y/2)
  qy = cos(r/2) * sin(p/2) * cos(y/2) + sin(r/2) * cos(p/2) * sin(y/2)
  qz = cos(r/2) * cos(p/2) * sin(y/2) - sin(r/2) * sin(p/2) * cos(y/2)
  return @SVector [qw, qx, qy, qz]
end

"""
The definition of quaternion here is
q = [w, x, y, z]
where 
w = cos(a/2)
x = v_x sin(a/2)
y = v_y sin(a/2)
z = v_z sin(a/2)
and v is axis unit vector

* Note that this definition is different from the one of ROS (w = [x, y, z, w])

"""

mutable struct Quaternion  
  vec::SVector{4, Float64}
  mat::SMatrix{3, 3, Float64}
end

function Quaternion(x, y, z, w)
  vec = @SVector [x, y, z, w]
  mat = quaternion2matrix(vec)
  Quaternion(vec, mat)
end

function Quaternion(r, p, y)
  vec = rpy2quaternion(r, p, y)
  mat = quaternion2matrix(vec)
  Quaternion(vec, mat)
end

function Quaternion(axis, angle)
    half_angle = angle*0.5
    vec_ = [cos(half_angle), 
            axis[1]*sin(half_angle), 
            axis[2]*sin(half_angle), 
            axis[2]*sin(half_angle)]
    vec = SVector{4}(vec_)
    mat = quaternion2matrix(vec)
    Quaternion(vec, mat)
end

function Base.:*(p::Quaternion, q::Quaternion)
  q0, q1, q2, q3 = q.vec
  mat = @SMatrix [q0 -q1 -q2 -q3;
                  q1 q0 -q3 q2;
                  q2 q3 q0 -q1;
                  q3 -q2 q1 q0]
  vec_new = mat * p.vec
  mat = quaternion2matrix(vec_new)
  Quaternion(vec_new, mat)
end

function (q::Quaternion)(x) 
  q.mat * x 
end

struct Transform
  trans::SVector{3, Float64}
  rot::Quaternion
end

function Transform(trans_, rotlike)
  trans = SVector{3, Float64}(trans_)

  rot = nothing
  if length(rotlike) == 3 # rpy case 
    r, p, y = rotlike
    rot = Quaternion(r, p, y)
  elseif length(rotlike) == 4 # quaternion case
    w, x, y, z = rotlike
    rot = Quaternion(w, x, y, z)
  else
    error("rotlike must be dim=3 or 4")
  end
  Transform(trans, rot)
end

function Transform(trans_, axis, angle)
  trans = SVector{3, Float64}(trans_)
  rot = Quaternion(axis, angle)
  Transform(trans, rot)
end

function (tf::Transform)(x)
  return tf.trans + tf.rot(x)
end

function Base.:∘(tf12::Transform, tf23::Transform)
  trans_new = tf23.trans + tf23.rot(tf12.trans)
  rot_new = tf12.rot * tf23.rot
  Transform(trans_new, rot_new)
end
